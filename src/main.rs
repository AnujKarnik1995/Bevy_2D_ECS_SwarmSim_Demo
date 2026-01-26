use bevy::prelude::*;
use ron::de::from_reader;
use std::fs::File;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use std::time::Duration;


#[cfg(feature = "headless")]
use bevy::app::ScheduleRunnerPlugin;

mod components;
mod systems;
mod resources;
mod utilityfunctions;

use systems::*;
use resources::SimulationConfig;

fn main() {
    // 1. Load the Config File from disk
    let config_path = "assets/simulation.ron";
    let file = File::open(config_path).expect("Failed to open config file");
    let config: SimulationConfig = from_reader(file).expect("Failed to parse config file");

    println!("Loaded Config: {:?}", config);

    let mut app = App::new();

    // ========================================================================
    // PART A: CHOOSE THE ENGINE MODE
    // ========================================================================
    
    // OPTION 1: VISUAL MODE (Default)
    // Runs if you DO NOT pass the 'headless' feature flag.
    #[cfg(not(feature = "headless"))] 
    {
        app.add_plugins(DefaultPlugins);
    }

    // OPTION 2: HEADLESS MODE (Server/CI/WSL Fix)
    // Runs only if you use --features headless
    #[cfg(feature = "headless")]
    {
        // MinimalPlugins gives you the Event Loop + Time + ECS.
        // It does NOT give you a Window, GPU Renderer, or Audio.
        app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(
            // We force the loop to run at 60 ticks per second.
            // Without this, Headless mode runs at 100% CPU speed (10,000+ FPS),
            // which is great for stress testing but hard to read logs.
            Duration::from_secs_f64(0.0),
        )));
        
        // Note: If you use AssetServer later, you might need:
        // app.add_plugins(AssetPlugin::default());
    }

    // ========================================================================
    // PART B: ADD COMMON RESOURCES & PLUGINS
    // ========================================================================
    app.insert_resource(config)
        .insert_resource(Time::<Fixed>::from_hz(60.0))
       .add_plugins(FrameTimeDiagnosticsPlugin::default())
       .add_plugins(LogDiagnosticsPlugin::default());

    // ========================================================================
    // PART C: ADD YOUR SYSTEMS
    // ========================================================================
    app.add_systems(Startup, setup_simulation)
       .add_systems(FixedUpdate, (
            movement_system, 
            robot_state_machine, 
            battery_system
       ))
       .add_systems(Update, 
            log_performance
            // camera_controls
        );
    
    

    // 3. Launch
    app.run();
}

// A manual system to print performance metrics
fn log_performance (time: Res<Time>, mut frame_count: Local<u32>) {
    *frame_count += 1;
    // Only print every 1000 frames to avoid spamming console IO
    if *frame_count % 1000 == 0 {
        // Calculate raw FPS (1.0 / delta_seconds)
        let fps = 1.0 / time.delta_secs();
        println!("🚀 Simulation Speed: {:.2} Ticks Per Second (Delta: {:.4}ms)", 
            fps, 
            time.delta_secs() * 1000.0
        );
    }
}