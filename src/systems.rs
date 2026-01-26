use bevy::prelude::*;
// use bevy::input::mouse::{MouseMotion, MouseWheel};

use crate::components::*;
use crate::resources::SimulationConfig;
use crate::utilityfunctions::*;

// --- SETUP ---

pub fn setup_simulation(mut commands: Commands, config: Res<SimulationConfig>) 
{
    // let center_x = (config.robot_count as f32 * 100.0) / 2.0;
    // // Camera
    commands.spawn((
        Camera2d, 
        Transform::from_xyz(-30.0, -430.0, 0.0)
    ));

    // Pickups
    for (x, y) in &config.pickup_stations 
    {
        commands.spawn((
            Sprite::from_color(Color::srgb(0.0, 1.0, 0.0), 
            Vec2::new(40.0, 40.0)), 
            Transform::from_xyz(*x, *y, 0.0), 
            PickupStation, 
            Booked(false)
        ));
    }
    // Dropoffs
    for (x, y) in &config.dropoff_stations 
    {
        commands.spawn((
            Sprite::from_color(Color::srgb(0.0, 0.0, 1.0), 
            Vec2::new(40.0, 40.0)), 
            Transform::from_xyz(*x, *y, 0.0), 
            DropoffStation, 
            Booked(false)
        ));
    }
    // Chargers
    for (x, y) in &config.charger_stations 
    {
        commands.spawn((
            Sprite::from_color(Color::srgb(1.0, 1.0, 0.0), 
            Vec2::new(50.0, 50.0)), 
            Transform::from_xyz(*x, *y, 0.0), 
            ChargerStation, 
            Booked(false)
        ));
    }

    // Robots
    for i in 0..config.robot_count 
    {
        commands.spawn((
            Sprite::from_color(Color::WHITE, 
            Vec2::new(30.0, 30.0)),
            Transform::from_xyz(0.0 + (i as f32 * 100.0), 50.0, 0.0),
            Robot,
            Speed(config.robot_speed),
            TargetPosition(Vec3::ZERO), 
            RobotState::Idle,
            RobotTimers {
                work: Timer::from_seconds(1.0, TimerMode::Once),
                charge: Timer::from_seconds(config.charging_time, TimerMode::Once),
            }, 
            ReservedStation(None),
            Battery(100.0), 
            SavedMemory(None) 
        ));
    }
}

// --- LOGIC ---

pub fn movement_system(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut param_set: ParamSet<(
        Query<(Entity, &Transform), With<Robot>>,
        Query<(Entity, &mut Transform, &TargetPosition, &Speed, &RobotState)>
    )>
) 
{
    // 1. Snapshot all obstacles
    // Optimization: explicitly reserve capacity if you know N to avoid re-allocations
    let obstacle_count = param_set.p0().iter().len();
    let obstacles: Vec<(Entity, Vec3)> = param_set.p0().iter()
        .map(|(e, t)| (e, t.translation))
        .collect(); // Note: vectors allocate, doing this every frame is costly for huge N

    // 2. Update robots
    // FIX: Add '_entity' if you aren't using it, but here you ARE using it in calculate_avoidance_force.
    // If you still get a warning, it means calculate_avoidance_force isn't using the argument.
    for (entity, mut transform, target, speed, state) in param_set.p1().iter_mut() {    
        
        // Skip dead robots
        if *state == RobotState::Dead { continue; }

        // Filter moving states
        let should_move = matches!(state, 
            RobotState::MovingToPickup | 
            RobotState::MovingToDropoff | 
            RobotState::MovingToCharger
        );
        if !should_move { continue; }

        let current_pos = transform.translation;

        // Collision avoidance
        let (separation_vector, critical_overlap) = calculate_avoidance_force(
            entity, 
            current_pos, 
            &obstacles, 
            config.collision_radius
        );

        // A. Goal Vector
        let target_vec = target.0 - current_pos;
        let dist_to_target = target_vec.length();
        
        // Logic Check: Don't normalize if already at target
        let goal_dir = if dist_to_target > 0.01 { 
            target_vec.normalize() 
        } else { 
            Vec3::ZERO 
        };

        // B. Apply Forces
        // FIX: Initialize directly from the if/else block to silence the warning
        let final_direction = if critical_overlap {
            // Emergency avoidance: Ignore goal, just run away
            separation_vector
        } else {
            // Standard Navigation: Mix goal and avoidance
            goal_dir + separation_vector
        };

        // C. Move
        // We use length_squared() because it's faster (no square root)
        if final_direction.length_squared() > 0.0001 {
            let move_dir = final_direction.normalize();
            
            // OPTIONAL: Simple Arrival Logic (Slow down when close)
            // Prevents the robot from jittering back and forth over the target
            let current_speed = if dist_to_target < 10.0 {
                speed.0 * (dist_to_target / 10.0).clamp(0.1, 1.0)
            } else {
                speed.0
            };

            transform.translation += move_dir * speed.0 * time.delta_secs();
        }
    }
}

// --- STATE MACHINE ---
pub fn robot_state_machine(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut robot_query: Query<(Entity, &mut RobotState, &mut TargetPosition, &Transform, &mut RobotTimers, &mut ReservedStation, &mut Battery, &mut SavedMemory), With<Robot>>,
    mut pickup_query: Query<(Entity, &Transform, &mut Booked), (With<PickupStation>, Without<DropoffStation>, Without<ChargerStation>)>,
    mut dropoff_query: Query<(Entity, &Transform, &mut Booked), (With<DropoffStation>, Without<PickupStation>, Without<ChargerStation>)>,
    mut charger_query: Query<(Entity, &Transform, &mut Booked), (With<ChargerStation>, Without<PickupStation>, Without<DropoffStation>)>
) 
{
    for (robot_entity, mut state, mut target, transform, mut timer, mut reserved, mut battery, mut memory) in &mut robot_query {
        
        if *state == RobotState::Dead { continue; }

        match *state {
            // (Idle, MovingToPickup, PickingUp, WaitingForDropoff, MovingToDropoff, DroppingOff)
            RobotState::Idle => 
            {
                 for (pickup_entity, pickup_transform, mut booked) in &mut pickup_query 
                 {
                    if !booked.0 
                    {
                        booked.0 = true; 
                        *state = RobotState::MovingToPickup;
                        target.0 = pickup_transform.translation;
                        reserved.0 = Some(pickup_entity);
                        break; 
                    }
                }
            }
            RobotState::MovingToPickup => 
            {
                if transform.translation.distance(target.0) < config.state_change_radius 
                {
                    *state = RobotState::PickingUp;
                    timer.work.reset(); 
                }
            }
            RobotState::PickingUp => 
            {
                timer.work.tick(time.delta());
                if timer.work.just_finished() 
                {
                    if let Some(station_entity) = reserved.0 
                    {
                        if let Ok((_, _, mut booked)) = pickup_query.get_mut(station_entity) 
                        {
                            booked.0 = false;
                        }
                    }
                    reserved.0 = None;
                    *state = RobotState::WaitingForDropoff;
                }
            }
            RobotState::WaitingForDropoff => 
            {
                for (dropoff_entity, dropoff_transform, mut booked) in &mut dropoff_query 
                {
                    if !booked.0 
                    {
                        booked.0 = true; 
                        *state = RobotState::MovingToDropoff;
                        target.0 = dropoff_transform.translation;
                        reserved.0 = Some(dropoff_entity);
                        break; 
                    }
                }
            }
            RobotState::MovingToDropoff => 
            {
                if transform.translation.distance(target.0) < config.state_change_radius 
                {
                    *state = RobotState::DroppingOff;
                    timer.work.reset();
                }
            }
            RobotState::DroppingOff => 
            {
                timer.work.tick(time.delta());
                if timer.work.just_finished() 
                {
                    if let Some(station_entity) = reserved.0 
                    {
                        if let Ok((_, _, mut booked)) = dropoff_query.get_mut(station_entity) 
                        {
                            booked.0 = false;
                        }
                    }
                    reserved.0 = None;
                    *state = RobotState::Idle; 
                }
            }

            RobotState::WaitingForCharger => 
            {
                for (charger_entity, charger_transform, mut booked) in &mut charger_query 
                {
                     if !booked.0 
                     {
                        booked.0 = true;
                        *state = RobotState::MovingToCharger;
                        target.0 = charger_transform.translation;
                        // We overwrite reserved.0 with the Charger ID temporarily.
                        // This is fine because we cached the old ID in 'memory'.
                        reserved.0 = Some(charger_entity); 
                        break;
                     }
                }
            }

            RobotState::MovingToCharger => 
            {
                if transform.translation.distance(target.0) < config.state_change_radius
                {
                    *state = RobotState::Charging;
                    timer.charge.reset();
                }
            }
            RobotState::Charging => 
            {
                timer.charge.tick(time.delta());
                
                battery.0 += 25.0 * time.delta_secs(); 
                if battery.0 > 100.0 { battery.0 = 100.0; }

                if timer.charge.just_finished() 
                {
                    // 1. Release the Charger
                    if let Some(station_entity) = reserved.0 
                    {
                        if let Ok((_, _, mut booked)) = charger_query.get_mut(station_entity) 
                        {
                            booked.0 = false;
                        }
                    }
                    
                    // 2. Resume memory (with key)
                    if let Some((saved_state, saved_target, saved_key)) = memory.0 
                    {
                        println!("Charged! Resuming {:?}...", saved_state);
                        *state = saved_state;
                        target.0 = saved_target;
                    
                        // when bot arrives at the station, it will have the correct ID to unlock it.
                        reserved.0 = saved_key; 
                    } 
                    else 
                    {
                        *state = RobotState::Idle;
                        reserved.0 = None;
                    }
                    memory.0 = None;
                }
            }
            RobotState::Dead => {}
        }
    }
}

// --- BATTERY SYSTEM ---
pub fn battery_system(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut query: Query<(&mut Battery, &mut Sprite, &mut RobotState, &mut SavedMemory, &TargetPosition, &ReservedStation), With<Robot>>
) 
{
    for (mut battery, mut sprite, mut state, mut memory, target, reserved) in &mut query {
        
        if *state == RobotState::Dead || *state == RobotState::Charging {
            continue;
        }

        let is_moving = match *state {
            RobotState::MovingToPickup | RobotState::MovingToDropoff | RobotState::MovingToCharger => true,
            _ => false
        };

        let drain_rate = if is_moving { config.drain_move } else { config.drain_idle };
        battery.0 -= drain_rate * time.delta_secs();

        // Death Check
        if battery.0 < config.dead_battery_threshold {
            *state = RobotState::Dead;
            sprite.color = Color::BLACK; 
            println!("Robot Died!");
            continue;
        }

        // Low Battery Check
        let charging_related = match *state {
            RobotState::WaitingForCharger | RobotState::MovingToCharger | RobotState::Charging => true,
            _ => false
        };

        if battery.0 < config.low_battery_threshold && !charging_related {
            let resume_state = match *state {
                RobotState::PickingUp => RobotState::MovingToPickup,
                RobotState::DroppingOff => RobotState::MovingToDropoff,
                _ => *state,
            };

            memory.0 = Some((resume_state, target.0, reserved.0));
            *state = RobotState::WaitingForCharger;
            println!("Low Battery! Finding charger...");
        }

        // Color Update
        if battery.0 > 50.0 {
            sprite.color = Color::WHITE;
        } else {
            sprite.color = Color::srgb(1.0, 0.5, 0.0); // Orange
        }
    }
} 

// // --- CAMERA CONTROLS ---
// pub fn camera_controls(
//     mut motion_events: EventReader<MouseMotion>,
//     mut scroll_events: EventReader<MouseWheel>,
//     mouse_buttons: Res<ButtonInput<MouseButton>>,
//     mut query: Query<(&mut Transform, &mut Projection), With<Camera>>,
// ) {
//     if let Ok((mut transform, mut projection)) = query.single_mut() {
//         if let Projection::Orthographic(ortho) = &mut *projection {
//             if mouse_buttons.pressed(MouseButton::Right) {
//                 for ev in motion_events.read() {
//                     transform.translation.x -= ev.delta.x * ortho.scale;
//                     transform.translation.y += ev.delta.y * ortho.scale;
//                 }
//             }

//             for ev in scroll_events.read() {
//                 ortho.scale = (ortho.scale - ev.y * 0.1).clamp(0.1, 5.0);
//             }
//         }
//     }
// }