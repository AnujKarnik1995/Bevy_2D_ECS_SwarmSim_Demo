use bevy::prelude::*;
use std::fs::File;
use ron::de::from_reader;

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

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(config) 
        .add_systems(Startup, setup_simulation)
        .add_systems(Update, (movement_system, robot_state_machine, battery_system))
        .run();
}