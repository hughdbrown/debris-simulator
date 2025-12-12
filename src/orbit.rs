//! Orbital mechanics and propagation

use crate::types::{OrbitalState, Vector3};
use crate::math::MU_EARTH;

// Minimum distance to avoid singularity (100 meters in km)
const MIN_DISTANCE_KM: f32 = 0.1;

/// Compute gravitational acceleration at a given position
/// a = -μ * r / |r|³
fn gravitational_acceleration(position: &Vector3) -> Vector3 {
    let r_mag = position.magnitude();

    // Protect against singularity at origin using safe threshold
    if r_mag < MIN_DISTANCE_KM || !r_mag.is_finite() {
        return Vector3::ZERO;
    }

    let r_cubed = r_mag * r_mag * r_mag;
    position.scale(-MU_EARTH / r_cubed)
}

/// State derivative for orbital motion
/// d/dt(r) = v
/// d/dt(v) = a = -μr/|r|³
struct StateDerivative {
    velocity: Vector3,
    acceleration: Vector3,
}

impl StateDerivative {
    fn compute(position: &Vector3, velocity: &Vector3) -> Self {
        Self {
            velocity: *velocity,
            acceleration: gravitational_acceleration(position),
        }
    }
}

/// RK4 integration step for orbital propagation
pub fn propagate_orbit(state: &OrbitalState, dt_seconds: f32) -> OrbitalState {
    let r0 = state.position;
    let v0 = state.velocity;

    // RK4 coefficients
    let k1 = StateDerivative::compute(&r0, &v0);

    let r1 = r0 + k1.velocity.scale(dt_seconds * 0.5);
    let v1 = v0 + k1.acceleration.scale(dt_seconds * 0.5);
    let k2 = StateDerivative::compute(&r1, &v1);

    let r2 = r0 + k2.velocity.scale(dt_seconds * 0.5);
    let v2 = v0 + k2.acceleration.scale(dt_seconds * 0.5);
    let k3 = StateDerivative::compute(&r2, &v2);

    let r3 = r0 + k3.velocity.scale(dt_seconds);
    let v3 = v0 + k3.acceleration.scale(dt_seconds);
    let k4 = StateDerivative::compute(&r3, &v3);

    // Weighted average
    let new_position = r0 + (k1.velocity + k2.velocity.scale(2.0) + k3.velocity.scale(2.0) + k4.velocity).scale(dt_seconds / 6.0);
    let new_velocity = v0 + (k1.acceleration + k2.acceleration.scale(2.0) + k3.acceleration.scale(2.0) + k4.acceleration).scale(dt_seconds / 6.0);

    OrbitalState::new(
        new_position,
        new_velocity,
        state.epoch_ms + (dt_seconds * 1000.0) as u64,
    )
}

/// Create a circular orbit at given altitude (km)
pub fn circular_orbit(altitude_km: f32, inclination_rad: f32, raan_rad: f32) -> OrbitalState {
    use micromath::F32Ext;

    let radius = crate::math::EARTH_RADIUS + altitude_km;
    let velocity_mag = (MU_EARTH / radius).sqrt();

    // Position in orbital plane, then rotate by inclination
    let x = radius * raan_rad.cos();
    let y = radius * raan_rad.sin();

    let position = Vector3::new(
        x,
        y * inclination_rad.cos(),
        y * inclination_rad.sin(),
    );

    // Velocity perpendicular to position
    let velocity = Vector3::new(
        -velocity_mag * raan_rad.sin(),
        velocity_mag * raan_rad.cos() * inclination_rad.cos(),
        velocity_mag * raan_rad.cos() * inclination_rad.sin(),
    );

    OrbitalState::new(position, velocity, 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::EARTH_RADIUS;
    use micromath::F32Ext;

    #[test]
    fn test_circular_orbit_velocity() {
        // LEO at 400 km altitude
        let altitude = 400.0;
        let orbit = circular_orbit(altitude, 0.0, 0.0);

        let radius = EARTH_RADIUS + altitude;
        let expected_velocity = (MU_EARTH / radius).sqrt();
        let actual_velocity = orbit.velocity.magnitude();

        // Within 1% tolerance
        assert!((actual_velocity - expected_velocity).abs() / expected_velocity < 0.01);
    }

    #[test]
    fn test_propagate_maintains_energy() {
        // Start with circular orbit
        let orbit0 = circular_orbit(400.0, 0.0, 0.0);

        // Compute specific energy: E = v²/2 - μ/r
        let energy0 = orbit0.velocity.magnitude_squared() * 0.5
                    - MU_EARTH / orbit0.position.magnitude();

        // Propagate for 60 seconds
        let orbit1 = propagate_orbit(&orbit0, 60.0);
        let energy1 = orbit1.velocity.magnitude_squared() * 0.5
                    - MU_EARTH / orbit1.position.magnitude();

        // Energy should be conserved (within numerical error)
        let energy_error = (energy1 - energy0).abs() / energy0.abs();
        assert!(energy_error < 0.001, "Energy error: {}", energy_error);
    }
}
