// Subscribes to color_image and publishes a Twist to cmd_vel on each received frame.
// Prints FPS every second.
//
// Topic names are injected via CLI args by the Python NativeModule system.

use dimos_native_module::LcmModule;
use lcm_msgs::geometry_msgs::{Twist, Vector3};
use lcm_msgs::sensor_msgs::Image;
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    let mut module = LcmModule::from_args().await.expect("Failed to create LCM module");

    let mut color_image = module.input("color_image", Image::decode);
    let cmd_vel = module.output("cmd_vel", Twist::encode);
    let _handle = module.spawn();

    println!("Listening on '{}'", color_image.topic);
    println!("Publishing to '{}'", cmd_vel.topic);
    println!("Press Ctrl+C to stop.\n");

    let twist = Twist {
        linear: Vector3 { x: 0.3, y: 0.0, z: 0.0 },
        angular: Vector3 { x: 0.0, y: 0.0, z: 0.5 },
    };

    let mut frame_count = 0u32;
    let mut last_print = Instant::now();

    loop {
        tokio::select! {
            Some(_frame) = color_image.recv() => {
                frame_count += 1;
                cmd_vel.publish(&twist).await.ok();

                if last_print.elapsed() >= Duration::from_secs(1) {
                    println!("FPS: {frame_count}");
                    frame_count = 0;
                    last_print = Instant::now();
                }
            }
        }
    }
}
