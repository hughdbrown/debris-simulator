//! Mathematical constants and utilities for orbital mechanics

/// Standard gravitational parameter for Earth (km³/s²)
pub const MU_EARTH: f32 = 398600.4418;

/// Earth radius (km)
pub const EARTH_RADIUS: f32 = 6378.137;

/// Converts degrees to radians
pub fn deg_to_rad(deg: f32) -> f32 {
    deg * core::f32::consts::PI / 180.0
}

/// Converts radians to degrees
pub fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / core::f32::consts::PI
}

/// Clamps a value between min and max
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deg_to_rad() {
        let rad = deg_to_rad(180.0);
        assert!((rad - core::f32::consts::PI).abs() < 0.001);
    }

    #[test]
    fn test_rad_to_deg() {
        let deg = rad_to_deg(core::f32::consts::PI);
        assert!((deg - 180.0).abs() < 0.001);
    }

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clamp(-1.0, 0.0, 10.0), 0.0);
        assert_eq!(clamp(15.0, 0.0, 10.0), 10.0);
    }
}
