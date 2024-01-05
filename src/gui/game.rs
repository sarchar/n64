use std::time::Instant;

use crate::*;
use gui::{App, AppWindow};

pub struct Game {
    pipeline: wgpu::RenderPipeline,
    frame_count: u64,
    last_fps_time: Instant,
    fps: f64,
}

impl App for Game {
    fn create(appwnd: &AppWindow) -> Self {
        let shader = appwnd.device().create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("game.wgsl").into()),
        });

        let render_pipeline_layout = appwnd.device().create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Pipeline Layout"),
            bind_group_layouts: &[],
            push_constant_ranges: &[],
        });

        let render_pipeline = appwnd.device().create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: appwnd.surface_config().format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        Self {
            pipeline: render_pipeline,
            frame_count: 0,
            last_fps_time: Instant::now(),
            fps: 0.0,
        }
    }

    fn update(&mut self, _appwnd: &AppWindow, _delta_time: f32) {
        self.frame_count += 1;
        if (self.frame_count % 5) == 0 {
            self.fps = 5.0 / self.last_fps_time.elapsed().as_secs_f64();
            self.last_fps_time = Instant::now();
        }
    }

    fn render(&mut self, appwnd: &AppWindow, view: &wgpu::TextureView) {
        let mut encoder: wgpu::CommandEncoder =
            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Game Encoder") });
        {
            let clear_color = wgpu::Color { r: 0.1, g: 0.2, b: 0.3, a: 1.0 };
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Game Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(clear_color),
                        store: true, //. wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                //.occlusion_query_set: None,
                //.timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.pipeline);
            render_pass.draw(0..3, 0..1);
        }
        appwnd.queue().submit(Some(encoder.finish()));
    }

    fn render_ui(&mut self, _appwnd: &AppWindow, ui: &imgui::Ui) {
        let window = ui.window("Stats");
        window.size([300.0, 100.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::Once)
              .build(|| {
                  ui.text(format!("FPS: {}", self.fps))
              });
    }
}
