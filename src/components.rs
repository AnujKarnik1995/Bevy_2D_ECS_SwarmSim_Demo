use bevy::prelude::*;

// --- STATES ---
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    Idle,
    MovingToPickup,
    PickingUp,
    WaitingForDropoff,
    MovingToDropoff,
    DroppingOff,
    WaitingForCharger,
    MovingToCharger,
    Charging,
    Dead,
}

// --- TAGS ---
#[derive(Component)]
pub struct Robot;

#[derive(Component)]
pub struct PickupStation;

#[derive(Component)]
pub struct DropoffStation;

#[derive(Component)]
pub struct ChargerStation;

// --- DATA ---
#[derive(Component)]
pub struct Speed(pub f32);

#[derive(Component)]
pub struct TargetPosition(pub Vec3);

#[derive(Component)]
pub struct RobotTimers {
    pub work: Timer,   // For Picking Up / Dropping Off (1.0s)
    pub charge: Timer, // For Charging (variable from config)
}

#[derive(Component)]
pub struct Booked(pub bool);

#[derive(Component)]
pub struct ReservedStation(pub Option<Entity>);

#[derive(Component)] 
pub struct Battery(pub f32); // battery level 0.0 to 100.0

#[derive(Component)]
pub struct SavedMemory(pub Option<(RobotState, Vec3, Option<Entity>)>); // stores last action to return to after charging completes
