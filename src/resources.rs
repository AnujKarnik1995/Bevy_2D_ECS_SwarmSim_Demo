use bevy::prelude::*;
use serde::Deserialize;

#[derive(Resource, Deserialize, Debug, Clone)]
pub struct SimulationConfig {
    pub robot_count: usize,
    pub robot_speed: f32,
    pub collision_radius: f32,
    pub state_change_radius: f32,
    
    // battery params
    pub low_battery_threshold: f32,
    pub dead_battery_threshold: f32,
    pub drain_idle: f32,
    pub drain_move: f32,
    pub charging_time: f32,

    pub pickup_stations: Vec<(f32, f32)>,
    pub dropoff_stations: Vec<(f32, f32)>,
    pub charger_stations: Vec<(f32, f32)>,
}