use bevy::prelude::*;

/// Calculates the separation force based on nearby obstacles (Boids logic).
/// Returns a tuple: (Separation Force Vector, Critical Overlap Boolean)
pub fn calculate_avoidance_force(
    current_entity: Entity,
    current_pos: Vec3,
    obstacles: &[(Entity, Vec3)], // Pass a slice of obstacles
    collision_radius: f32,
) -> (Vec3, bool) {
    let mut separation_vector = Vec3::ZERO;
    let mut critical_overlap = false;

    for (other_entity, other_pos) in obstacles 
    {
        // Skip self
        if current_entity == *other_entity 
        {
            continue;
        }

        let distance = current_pos.distance(*other_pos);

        if distance < collision_radius 
        {
            let away_direction = (current_pos - *other_pos).normalize();
            // The closer they are, the stronger the force (0.0 to 1.0)
            let strength = 1.0 - (distance / collision_radius);

            // Tie-breaker logic: Lower ID yields (moves away faster), Higher ID stays course
            if current_entity < *other_entity 
            {
                //"yielder" - move away slightly
                separation_vector += away_direction * strength * 0.5;
            } else 
            {
                if distance < (collision_radius * 0.5) {
                    critical_overlap = true;
                }
                separation_vector += away_direction * strength * 3.0;
            }
        }
    }
    (separation_vector, critical_overlap)
}

