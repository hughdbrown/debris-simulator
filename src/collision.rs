//! Collision detection and risk assessment

use crate::types::{CollisionRisk, DebrisObject, OrbitalState, RiskLevel, Vector3};

/// Calculate time to closest approach (TCA) between two objects
/// Returns (TCA in seconds, miss distance in km)
pub fn calculate_tca(own_ship: &OrbitalState, debris: &OrbitalState) -> (f32, f32) {
    let r_rel = debris.position - own_ship.position;
    let v_rel = debris.velocity - own_ship.velocity;

    // TCA occurs when d/dt(|r_rel(t)|²) = 0
    // |r_rel(t)|² = |r_rel0 + v_rel*t|²
    // d/dt = 2*(r_rel0 + v_rel*t)·v_rel = 0
    // t = -(r_rel0·v_rel) / |v_rel|²

    let v_rel_squared = v_rel.magnitude_squared();

    if v_rel_squared < 1e-6 {
        // Objects moving together - use current distance
        return (0.0, r_rel.magnitude());
    }

    let tca_seconds = -(r_rel.dot(&v_rel)) / v_rel_squared;

    // Don't consider negative TCA (already passed closest approach)
    let tca_seconds = if tca_seconds < 0.0 { 0.0 } else { tca_seconds };

    // Calculate miss distance at TCA
    let r_at_tca = r_rel + v_rel.scale(tca_seconds);
    let miss_distance = r_at_tca.magnitude();

    (tca_seconds, miss_distance)
}

// Risk assessment thresholds (constants for clarity)
const HIGH_RISK_DISTANCE_KM: f32 = 1.0;
const HIGH_RISK_TIME_S: f32 = 600.0;
const MEDIUM_RISK_DISTANCE_KM: f32 = 5.0;
const MEDIUM_RISK_TIME_S: f32 = 1200.0;
const LOW_RISK_DISTANCE_KM: f32 = 10.0;
const LOW_RISK_TIME_S: f32 = 3600.0;
const MAX_TCA_U32: f32 = u32::MAX as f32;

/// Assess collision risk based on TCA and miss distance
pub fn assess_risk(debris: &DebrisObject, own_ship: &OrbitalState) -> CollisionRisk {
    let (tca_seconds, miss_distance_km) = calculate_tca(own_ship, &debris.state);

    // Validate inputs
    let tca_seconds = if tca_seconds.is_finite() && tca_seconds >= 0.0 {
        tca_seconds.min(MAX_TCA_U32)
    } else {
        MAX_TCA_U32  // Treat invalid TCA as far future
    };

    let miss_distance_km = if miss_distance_km.is_finite() && miss_distance_km >= 0.0 {
        miss_distance_km
    } else {
        f32::MAX  // Treat invalid distance as very far
    };

    // Risk thresholds
    let risk_level = if miss_distance_km < HIGH_RISK_DISTANCE_KM && tca_seconds < HIGH_RISK_TIME_S {
        RiskLevel::High
    } else if miss_distance_km < MEDIUM_RISK_DISTANCE_KM && tca_seconds < MEDIUM_RISK_TIME_S {
        RiskLevel::Medium
    } else if miss_distance_km < LOW_RISK_DISTANCE_KM && tca_seconds < LOW_RISK_TIME_S {
        RiskLevel::Low
    } else {
        RiskLevel::None
    };

    CollisionRisk::new(
        debris.id,
        tca_seconds as u32,  // Safe now after validation
        miss_distance_km,
        risk_level,
    )
}

/// Calculate collision avoidance maneuver
/// Uses simple impulsive delta-v perpendicular to relative velocity
pub fn calculate_avoidance_maneuver(
    own_ship: &OrbitalState,
    debris: &OrbitalState,
    desired_miss_distance_km: f32,
) -> Vector3 {
    let r_rel = debris.position - own_ship.position;
    let v_rel = debris.velocity - own_ship.velocity;

    // Direction perpendicular to relative velocity
    let normal = r_rel.cross(&v_rel);
    if normal.magnitude() < 1e-6 {
        // Collinear - use position vector
        return r_rel.normalized().scale(0.1); // 0.1 m/s safety maneuver
    }

    let maneuver_dir = normal.normalized();

    // Estimate delta-v needed (simplified)
    // For small maneuvers: Δd ≈ Δv * TCA
    let (tca_seconds, current_miss_distance) = calculate_tca(own_ship, debris);

    if tca_seconds < 1.0 {
        // Too late for maneuver
        return Vector3::ZERO;
    }

    let delta_distance = desired_miss_distance_km - current_miss_distance;
    let delta_v_kps = delta_distance / tca_seconds;

    // Convert to m/s and validate
    let delta_v_mps = delta_v_kps * 1000.0;

    // Ensure delta_v is finite before clamping
    if !delta_v_mps.is_finite() {
        return Vector3::ZERO;
    }

    let limited_dv = crate::math::clamp(delta_v_mps.abs(), 0.01, 10.0);

    maneuver_dir.scale(limited_dv * delta_v_mps.signum())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tca_head_on() {
        let ship = OrbitalState::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            0,
        );

        let debris = OrbitalState::new(
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            0,
        );

        let (tca, miss_distance) = calculate_tca(&ship, &debris);

        // Should meet at t=50s at the origin
        assert!((tca - 50.0).abs() < 0.1);
        assert!(miss_distance < 0.1);
    }

    #[test]
    fn test_tca_parallel() {
        let ship = OrbitalState::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            0,
        );

        let debris = OrbitalState::new(
            Vector3::new(0.0, 10.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            0,
        );

        let (tca, miss_distance) = calculate_tca(&ship, &debris);

        // Parallel trajectories - already at closest
        assert!(tca.abs() < 0.1);
        assert!((miss_distance - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_risk_assessment_high() {
        let ship = OrbitalState::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            0,
        );

        let debris = DebrisObject::new(
            1,
            OrbitalState::new(
                Vector3::new(100.0, 0.0, 0.0),
                Vector3::new(-1.0, 0.0, 0.0),
                0,
            ),
        );

        let risk = assess_risk(&debris, &ship);
        assert_eq!(risk.risk_level, RiskLevel::High);
    }
}
