use std::collections::HashMap;
use std::io;
use tokio::sync::mpsc;

use dimos_lcm::{Lcm, LcmOptions};

const INPUT_CHANNEL_CAPACITY: usize = 16;
const PUBLISH_CHANNEL_CAPACITY: usize = 64;

// Internal trait for dynamic dispatch in the recv loop.
// Each subscribe() call produces a TypedRoute that decodes
// its message type and forward it to the right channel (Input).
trait Route: Send {
    fn topic(&self) -> &str;
    fn try_dispatch(&self, data: &[u8]);
}

struct TypedRoute<T: Send + 'static> {
    topic: String,
    decode: fn(&[u8]) -> io::Result<T>,
    sender: mpsc::Sender<T>,
}

impl<T: Send + 'static> Route for TypedRoute<T> {
    fn topic(&self) -> &str {
        &self.topic
    }

    fn try_dispatch(&self, data: &[u8]) {
        match (self.decode)(data) {
            // this try_send defines the QoS of the lcm module
            // as is, if there are any pending messages on this topic, new ones are dropped
            Ok(msg) => { let _ = self.sender.try_send(msg); }
            Err(e) => eprintln!("dimos_module: decode error on {}: {e}", self.topic),
        }
    }
}

pub struct Input<T> {
    pub topic: String,
    // messages are sent to receiver from main lcm handle loop
    receiver: mpsc::Receiver<T>,
}

impl<T> Input<T> {
    pub async fn recv(&mut self) -> Option<T> {
        self.receiver.recv().await
    }
}

pub struct Output<T> {
    pub topic: String,
    encode: fn(&T) -> Vec<u8>,
    sender: mpsc::Sender<(String, Vec<u8>)>,
}

impl<T> Output<T> {
    pub async fn publish(&self, msg: &T) -> io::Result<()> {
        let data = (self.encode)(msg);
        self.sender
            .send((self.topic.clone(), data))
            .await
            .map_err(|_| io::Error::new(io::ErrorKind::BrokenPipe, "background task gone"))
    }
}

/// High-level wrapper around the LCM transport for use in dimos native modules.
///
/// Topic names are injected at runtime via CLI args from the Python NativeModule
/// system (`--port_name /topic#type.Name`). Call `from_args()` to parse these,
/// then declare ports with `input()`/`output()` before starting the background
/// task with `spawn()`.
pub struct LcmModule {
    lcm: Lcm,
    routes: Vec<Box<dyn Route>>,
    topics: HashMap<String, String>,
    publish_tx: mpsc::Sender<(String, Vec<u8>)>,
    publish_rx: mpsc::Receiver<(String, Vec<u8>)>,
}

impl LcmModule {
    pub async fn new() -> io::Result<Self> {
        Self::with_options(LcmOptions::default()).await
    }

    pub async fn with_options(opts: LcmOptions) -> io::Result<Self> {
        let lcm = Lcm::with_options(opts).await?;
        let (publish_tx, publish_rx) = mpsc::channel(PUBLISH_CHANNEL_CAPACITY);
        Ok(Self {
            lcm,
            routes: Vec::new(),
            topics: HashMap::new(),
            publish_tx,
            publish_rx,
        })
    }

    /// Parse `--port_name topic_string` pairs from argv, as injected by NativeModule.
    pub async fn from_args() -> io::Result<Self> {
        let mut module = Self::new().await?;
        let args: Vec<String> = std::env::args().collect();
        let mut i = 1;
        while i < args.len() {
            if let Some(port) = args[i].strip_prefix("--") {
                if i + 1 < args.len() && !args[i + 1].starts_with("--") {
                    module.topics.insert(port.to_string(), args[i + 1].clone());
                    i += 2;
                    continue;
                }
            }
            i += 1;
        }
        Ok(module)
    }

    /// Manually set a topic for a port if launching without cli args.
    pub fn map_topic(&mut self, port: &str, topic: &str) {
        self.topics.insert(port.to_string(), topic.to_string());
    }

    fn topic_for(&self, port: &str) -> String {
        self.topics
            .get(port)
            .cloned()
            .unwrap_or_else(|| format!("/{port}"))
    }

    /// Register an input port. Must be called before `spawn()`.
    pub fn input<T: Send + 'static>(
        &mut self,
        port: &str,
        decode: fn(&[u8]) -> io::Result<T>,
    ) -> Input<T> {
        let topic = self.topic_for(port);
        let (tx, rx) = mpsc::channel(INPUT_CHANNEL_CAPACITY);
        self.routes.push(Box::new(TypedRoute { topic: topic.clone(), decode, sender: tx }));
        Input { topic, receiver: rx }
    }

    /// Register an output port. Must be called before `spawn()`.
    pub fn output<T: Send + 'static>(
        &self,
        port: &str,
        encode: fn(&T) -> Vec<u8>,
    ) -> Output<T> {
        Output {
            topic: self.topic_for(port),
            encode,
            sender: self.publish_tx.clone(),
        }
    }

    /// Start the background recv/dispatch/publish loop.
    ///
    /// Consumes the module — no new ports can be registered after this point.
    /// Dropping the returned handle detaches the task; it keeps running until
    /// the process exits.
    pub fn spawn(self) -> LcmModuleHandle {
        let LcmModule { mut lcm, routes, mut publish_rx, .. } = self;

        let handle = tokio::spawn(async move {
            loop {
                tokio::select! {
                    result = lcm.recv() => match result {
                        Ok(msg) => {
                            for route in &routes {
                                if route.topic() == msg.channel {
                                    route.try_dispatch(&msg.data);
                                }
                            }
                        }
                        Err(e) => eprintln!("dimos_module: recv error: {e}"),
                    },
                    Some((topic, data)) = publish_rx.recv() => {
                        if let Err(e) = lcm.publish(&topic, &data).await {
                            eprintln!("dimos_module: publish error on {topic}: {e}");
                        }
                    }
                }
            }
        });

        LcmModuleHandle(handle)
    }
}

pub struct LcmModuleHandle(tokio::task::JoinHandle<()>);

impl LcmModuleHandle {
    pub async fn join(self) -> Result<(), tokio::task::JoinError> {
        self.0.await
    }
}
