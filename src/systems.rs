use bevy::prelude::*;
use crate::components::*;
use crate::resources::SimulationConfig;
use crate::utilityfunctions::*;

// --- SETUP ---

pub fn setup_simulation(mut commands: Commands, config: Res<SimulationConfig>) 
{
    commands.spawn(Camera2d);

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
            WorkTimer(Timer::from_seconds(1.0, TimerMode::Once)), 
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
    // 1. Snapshot all obstacles (read-only access)
    let obstacles: Vec<(Entity, Vec3)> = param_set.p0().iter()
        .map(|(e, t)| (e, t.translation))
        .collect();

    // 2. Update robots (mutable access)
    for (entity, mut transform, target, speed, state) in param_set.p1().iter_mut() {    
        if *state == RobotState::Dead { continue; }

        let should_move = match state {
            RobotState::MovingToPickup | RobotState::MovingToDropoff | RobotState::MovingToCharger => true,
            _ => false,
        };
        if !should_move { continue; }

        let current_pos = transform.translation;

        // collision avoidance
        let (separation_vector, critical_overlap) = calculate_avoidance_force(
            entity, 
            current_pos, 
            &obstacles, 
            config.collision_radius
        );

        // A. Goal Vector
        let target_vec = target.0 - current_pos;
        let goal_dir = if target_vec.length() > 0.01 { target_vec.normalize() } else { Vec3::ZERO };

        // B. Apply Forces
        let mut final_direction = Vec3::ZERO;

        if critical_overlap 
        {
            // Emergency avoidance only
            final_direction = separation_vector;
        } 
        else 
        {
            // Mix goal and avoidance
            final_direction = goal_dir + separation_vector;
        }

        if final_direction.length() > 0.01 
        {
            transform.translation += final_direction.normalize() * speed.0 * time.delta_secs();
        }
    }
}

// --- STATE MACHINE ---
pub fn robot_state_machine(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    mut robot_query: Query<(Entity, &mut RobotState, &mut TargetPosition, &Transform, &mut WorkTimer, &mut ReservedStation, &mut Battery, &mut SavedMemory), With<Robot>>,
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
                    timer.0.reset(); 
                }
            }
            RobotState::PickingUp => 
            {
                timer.0.tick(time.delta());
                if timer.0.just_finished() 
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
                    timer.0.reset();
                }
            }
            RobotState::DroppingOff => 
            {
                timer.0.tick(time.delta());
                if timer.0.just_finished() 
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
                    timer.0 = Timer::from_seconds(config.charging_time, TimerMode::Once); 
                }
            }
            RobotState::Charging => 
            {
                timer.0.tick(time.delta());
                
                battery.0 += 25.0 * time.delta_secs(); 
                if battery.0 > 100.0 { battery.0 = 100.0; }

                if timer.0.just_finished() 
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
    // Note: We need 'reserved' here now to save it
    mut query: Query<(&mut Battery, &mut Sprite, &mut RobotState, &mut SavedMemory, &TargetPosition, &ReservedStation), With<Robot>>
) 
{
    for (mut battery, mut sprite, mut state, mut memory, target, reserved) in &mut query {
        
        if *state == RobotState::Dead || *state == RobotState::Charging 
        {
            continue;
        }

        let is_moving = match *state 
        {
            RobotState::MovingToPickup | RobotState::MovingToDropoff | RobotState::MovingToCharger => true,
            _ => false
        };

        let drain_rate = if is_moving { config.drain_move } else { config.drain_idle };
        battery.0 -= drain_rate * time.delta_secs();

        // Death
        if battery.0 < config.dead_battery_threshold 
        {
            *state = RobotState::Dead;
            sprite.color = Color::BLACK;
            println!("Robot Died!");
            continue;
        }

        // Low Battery Interrupt
        let charging_related = match *state 
        {
            RobotState::WaitingForCharger | RobotState::MovingToCharger | RobotState::Charging => true,
            _ => false
        };

        if battery.0 < config.low_battery_threshold && !charging_related 
        {
            // If bots are currently "Working" (PickingUp/DroppingOff), save the "MovingTo" state instead.
            // This forces the timer to reset cleanly when we return.
            let resume_state = match *state 
            {
                RobotState::PickingUp => RobotState::MovingToPickup,
                RobotState::DroppingOff => RobotState::MovingToDropoff,
                _ => *state,
            };

            memory.0 = Some((resume_state, target.0, reserved.0));
            
            *state = RobotState::WaitingForCharger;
            println!("Low Battery! Saving task (with key) and finding charger...");
        }

        // Color Coding
        if battery.0 > 50.0 
        {
            sprite.color = Color::WHITE; // Full
        } 
        else 
        {
            sprite.color = Color::srgb(1.0, 0.5, 0.0); // Low (Orange)
        }
    }
}