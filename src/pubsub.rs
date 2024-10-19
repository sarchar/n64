use std::{sync::{Arc, Mutex}, marker::PhantomData};

use anymap::{Map, any::Any};
use crossbeam::channel;

type AnyMap = Map<dyn Any + Send + Sync>;

#[derive(Clone)]
pub struct Pubsub {
    subscribers: Arc<Mutex<AnyMap>>,
    rx_channels: Arc<Mutex<AnyMap>>,
    updaters: Arc<Mutex<Vec<Box<dyn PubsubUpdaterTrait + Send + Sync + 'static>>>>,
}

impl Pubsub {
    pub fn new() -> Self {
        Self {
            subscribers: Arc::new(Mutex::new(AnyMap::new())),
            rx_channels: Arc::new(Mutex::new(AnyMap::new())),
            updaters   : Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn subscribe<M: Send + Sync + Clone + 'static>(&self) -> channel::Receiver<M> {
        let (tx, rx) = channel::unbounded();
        self.subscribers
            .lock()
            .unwrap()
            .entry::<Vec<channel::Sender<M>>>()
            .or_insert_with(|| {
                Vec::new()
            })
            .push(tx)
            ;
        rx
    }

    pub fn create_channel<M: Send + Sync + Clone + 'static>(&self) -> channel::Sender<M> {
        let (tx, rx) = channel::unbounded();
        self.rx_channels.lock().unwrap().insert::<channel::Receiver<M>>(rx);
        self.updaters.lock().unwrap().push(Box::new(PubsubUpdater::<M>::new()));
        tx
    }

    pub fn update(&mut self) {
        // use clone() to satisfy the borrow checker
        for updater in self.updaters.clone().lock().unwrap().iter() {
            updater.update(self);
        }
    }

    fn update_channel<M: Send + Sync + Clone + 'static>(&self) {
        if let anymap::Entry::Occupied(mut entry) = self.rx_channels.lock().unwrap().entry::<channel::Receiver<M>>() {
            while let Ok(msg) = entry.get_mut().try_recv() {
                if let anymap::Entry::Occupied(mut subs) = self.subscribers.lock().unwrap().entry::<Vec<channel::Sender<M>>>() {
                    subs.get_mut().retain_mut(|sub| sub.try_send(msg.clone()).is_ok());
                }
            }
        }
    }
}

// Tried doing something like having a dyn Trait encapsulate a 
trait PubsubUpdaterTrait {
    fn update(&self, pubsub: &mut Pubsub);
}

#[derive(Default)]
struct PubsubUpdater<M: Send + Sync + Clone + 'static> {
    _p: PhantomData<M>
}

impl<M: Sync + Send + Clone + 'static> PubsubUpdater<M> { 
    fn new() -> Self {
        Self {
            _p: PhantomData::default(),
        }
    }
}

impl<M: Sync + Send + Clone + 'static> PubsubUpdaterTrait for PubsubUpdater<M> { 
    fn update(&self, pubsub: &mut Pubsub) {
        pubsub.update_channel::<M>();
    }        
}
