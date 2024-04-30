#[allow(unused_imports)]
use tracing::{debug, error, trace, info, warn};

use crate::*;

use mips::{InterruptUpdate, InterruptUpdateMode, IMask_AI};

use rcp::DmaInfo;

#[derive()]
pub struct AudioInterface {
    comms: SystemCommunication,

    // communication channel to be notified when DMA is complete
    dma_completed_rx: mpsc::Receiver<DmaInfo>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,

    // register state
    dram_address: u32,
    transfer_length: Option<u32>, // when Some(), a DMA is pending because a current buffer is currently playing
    dma_enable: bool,
    count: u32,                   // internal DAC counter

    // buffer states
    buffer_playing: bool,
    next_buffer: Vec<u32>,
    sample_rate: u32,
    output_sample_rate: u32,

    // Audio buffer queue
    audio_queue: sdl2::audio::AudioQueue<f32>,
}

impl AudioInterface {
    pub fn new(comms: SystemCommunication) -> Self {
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        let sdl_context = sdl2::init().unwrap();
        let audio_subsystem = sdl_context.audio().unwrap();

        //for driver in sdl2::audio::drivers() {
        //    println!("SDL audio driver: {}", driver);
        //}

        // TODO make this configurable in the UI
        let desired_spec = sdl2::audio::AudioSpecDesired {
            freq: Some(48000),
            channels: Some(2),
            samples: Some(256),
        };

        let audio_queue = audio_subsystem.open_queue(None, &desired_spec).unwrap();
        let spec = audio_queue.spec();
        info!(target: "AI", "Initialized audio freq={} channels={} format={:?}", spec.freq, spec.channels, spec.format);

        Self {
            comms: comms,
            dma_completed_tx: dma_completed_tx,
            dma_completed_rx: dma_completed_rx,

            dram_address: 0,
            transfer_length: None,
            dma_enable: false,
            count: 0,

            buffer_playing: false,
            next_buffer: Vec::new(),
            sample_rate: 0,
            output_sample_rate: spec.freq as u32,

            audio_queue: audio_queue,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "AI", "reset");
        self.dram_address = 0;
        self.transfer_length = None;
        self.dma_enable = false;
        self.count = 0;
        self.buffer_playing = false;
    }

    pub fn step(&mut self) {
        while let Ok(_) = self.dma_completed_rx.try_recv() {}

        // return if we don't need to be playing audio
        if !self.dma_enable { return; }

        // if the next buffer has been transfered over, go ahead and queue it up
        // I don't think this will be a problem, but in the future we could check
        // self.audio_queue.size() to fall below a certain threshold before uploading the
        // buffer.  But I think games won't be queuing audio faster than their playback
        // framerate.
        if self.next_buffer.len() > 0 && self.sample_rate > 0 {
            // take the next buffer and replace it with an empty one
            let buffer = std::mem::replace(&mut self.next_buffer, Vec::new()); 

            // convert the i16's (which are packed as u32's) into f32
            // TODO maybe we can use output i16 format and let the hardware do the conversion
            // but we're not processing /that/ much data.
            let mut queue_buffer = Vec::new();
            for v in buffer.iter() {
                let mut left = ((v >> 16) as i16) as f32;
                left = if left < 0.0 { left / 32768.0 } else { left / 32767.0 };
                let mut right = ((v & 0xFFFF) as i16) as f32;
                right = if right < 0.0 { right / 32768.0 } else { right / 32767.0 };
                queue_buffer.push(left);
                queue_buffer.push(right);
            }

            // resample the game audio to fit our output device
            let resampled = samplerate::convert(self.sample_rate, self.output_sample_rate, 2, samplerate::ConverterType::SincBestQuality, &queue_buffer).unwrap();

            // queue the data into SDL
            self.audio_queue.queue_audio(&resampled).unwrap();

            // make sure playback is enabled
            self.play();

            //println!("queued up audio buffer sample count={}", buffer.len());
        } else {
            // there's no next buffer, technically audio could still be playing but we're not
            // queuing audio data any more
            self.buffer_playing = false;
        }

        // if a dma is pending, and no next buffer, start it and trigger AI interrupt
        if self.next_buffer.len() == 0 && self.transfer_length.is_some() {
            let len = std::mem::replace(&mut self.transfer_length, None).unwrap();

            let dma_info = DmaInfo {
                initiator     : "AI",
                source_address: (self.dram_address),
                dest_address  : 0x045F_0000, // Special hacky address to capture the audio buffer
                count         : 1,
                length        : len,
                source_stride : 0,
                completed     : Some(self.dma_completed_tx.clone()),
                ..Default::default()
            };

            // mark system as playing
            self.buffer_playing = true;

            // Send DMA
            self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();

            // AI interrupt triggers at the start of dma
            self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_AI, InterruptUpdateMode::SetInterrupt)).unwrap();
            
            // Interrupt cpu to process things
            self.comms.break_cpu();
        }
    }

    // tell SDL to start playing
    pub fn play(&self) {
        self.audio_queue.resume();
    }

    // tell SDL to stop
    pub fn pause(&self) {
        self.audio_queue.pause();
    }
}

impl Addressable for AudioInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "AI", "read32 address=${:08X}", offset);

        match offset {
            // AI_DRAM_ADDR - returns AI_LENGTH
            // AI_LENGTH
            // AI_CONTROL - returns AI_LENGTH
            0x0_0000 | 0x0_0004 | 0x0_0008 => {
                Ok(self.audio_queue.size() * (std::mem::size_of::<f32>() as u32))
            },

            // AI_STATUS
            0x0_000C => {
                let busy  = self.buffer_playing;
                let full  = busy && self.transfer_length.is_some();
                let en    = self.dma_enable;
                let wc    = false; // TODO
                let bc    = true; // TODO
                let count = self.count;
                Ok(((full as u32) << 31)
                   | ((busy as u32) << 30)
                   | ((en as u32) << 25)
                   | ((wc as u32) << 20)
                   | ((bc as u32) << 16)
                   | ((count & 0x3FFF) << 1)
                   | ((full as u32) << 0)
                   | 0x0110_0000)
            },

            _ => {
                todo!();
            },
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "AI", "write32 value=${:08X} address=${:08X}", value, offset);

        match offset {
            // AI_DRAM_ADDR
            0x0_0000 => {
                trace!(target: "AI", "write32 AI_DRAM_ADDR value=${:08X}", value);
                self.dram_address = value & 0x00FFFFF8;
            },

            // AI_LENGTH
            0x0_0004 => {
                trace!(target: "AI", "write32 AI_LENGTH value=${:08X} (self.transfer_length={:?})", value, self.transfer_length);
                let len = value & 0x0003_FFF8;

                if len == 0 {
                    self.transfer_length = None;
                } else {
                    match self.transfer_length {
                        Some(_) => { // overwrite queued length
                            self.transfer_length = Some(len);
                        },
                        None => { // No DMA pending, either queue or start dma
                            if self.buffer_playing || !self.dma_enable { // buffer playing or dma is disabled, queue dma
                                self.transfer_length = Some(len);
                                //println!("new length={:?}", self.transfer_length);
                            } else {
                                // no buffer playing, dma enabled, start transfer
                                let dma_info = DmaInfo {
                                    initiator     : "AI",
                                    source_address: (self.dram_address),
                                    dest_address  : 0x045F_0000, // Special hacky address to capture the audio buffer
                                    count         : 1,
                                    length        : len,
                                    source_stride : 0,
                                    completed     : Some(self.dma_completed_tx.clone()),
                                    ..Default::default()
                                };

                                // Set playing flag even though the dma hasn't technically started yet. this
                                // really means that a DMA is "in progress" and thus so is audio playback
                                self.buffer_playing = true;

                                // Send DMA
                                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();

                                // AI interrupt triggers at the start of dma
                                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_AI, InterruptUpdateMode::SetInterrupt)).unwrap();
                                
                                // Interrupt cpu to process things
                                self.comms.break_cpu();
                            }
                        }
                    }
                }
            },

            // AI_CONTROL
            0x0_0008 => {
                trace!(target: "AI", "write32 AI_CONTROL value=${:08X}", value);
                // if nothing is playing, reset state
                if !self.dma_enable && !self.buffer_playing {
                    self.transfer_length = None; // next write to AI_LENGTH triggers dma, which starts playback
                }
                self.dma_enable = (value & 0x01) == 0x01;
            },

            // AI_STATUS
            0x0_000C => {
                // this register is read only, and writes ACK interrupts
                trace!(target: "AI", "write32 AI_STATUS value=${:08X}", value);
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_AI, InterruptUpdateMode::ClearInterrupt)).unwrap();
            },

            // AI_DACRATE
            0x0_0010 => {
                trace!(target: "AI", "write32 AI_DACRATE value=${:08X}", value);
                self.sample_rate = 48726144 / (value + 1); // TODO needs to be adjusted to PAL?
                info!(target: "AI", "setting DAC sample rate to {}", self.sample_rate);
            },

            // AI_BITRATE
            0x0_0014 => {
                trace!(target: "AI", "write32 AI_BITRATE value=${:08X}", value);
                info!(target: "AI", "setting DAC bitrate to {}", 48726144 / (value + 1));
            },

            _ => {
                todo!();
            },
        }

        Ok(WriteReturnSignal::None)
    }

    fn write_block(&mut self, offset: usize, block: &[u32], _length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset == 0x000F_0000 {
            // Incoming buffer!
            //println!("got block: {:?}", block);
            assert!(self.next_buffer.len() == 0);
            self.next_buffer = block.to_owned();
        } else {
            panic!("should never happen");
        }
        Ok(WriteReturnSignal::None)
    }

}

