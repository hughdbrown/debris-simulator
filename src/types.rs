//! Core data types for the debris avoidance simulator

use defmt::Format;

/// 3D vector for position and velocity
#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn magnitude(&self) -> f32 {
        micromath::F32Ext::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn magnitude_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn normalized(&self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            Self {
                x: self.x / mag,
                y: self.y / mag,
                z: self.z / mag,
            }
        } else {
            *self
        }
    }

    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn scale(&self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl core::ops::Add for Vector3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl core::ops::Sub for Vector3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl core::ops::Mul<f32> for Vector3 {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        self.scale(scalar)
    }
}

/// Orbital state (position and velocity)
#[derive(Clone, Copy, Debug, Format)]
pub struct OrbitalState {
    pub position: Vector3,  // km
    pub velocity: Vector3,  // km/s
    pub epoch_ms: u64,      // milliseconds since start
}

impl OrbitalState {
    pub const fn new(position: Vector3, velocity: Vector3, epoch_ms: u64) -> Self {
        Self {
            position,
            velocity,
            epoch_ms,
        }
    }
}

/// Debris object with unique ID
#[derive(Clone, Copy, Debug, Format)]
pub struct DebrisObject {
    pub id: u8,
    pub state: OrbitalState,
}

impl DebrisObject {
    pub const fn new(id: u8, state: OrbitalState) -> Self {
        Self { id, state }
    }
}

/// Sensor reading from LIDAR/radar simulation
#[derive(Clone, Copy, Debug, Format)]
pub struct SensorReading {
    pub distance_km: f32,
    pub azimuth_deg: f32,
    pub elevation_deg: f32,
    pub timestamp_ms: u64,
}

impl SensorReading {
    pub const fn new(distance_km: f32, azimuth_deg: f32, elevation_deg: f32, timestamp_ms: u64) -> Self {
        Self {
            distance_km,
            azimuth_deg,
            elevation_deg,
            timestamp_ms,
        }
    }
}

/// Risk level for collision assessment
#[derive(Clone, Copy, Debug, Format, PartialEq, Eq)]
pub enum RiskLevel {
    None,
    Low,
    Medium,
    High,
}

/// Collision risk assessment
#[derive(Clone, Copy, Debug, Format)]
pub struct CollisionRisk {
    pub debris_id: u8,
    pub tca_seconds: u32,        // Time to closest approach
    pub miss_distance_km: f32,   // Distance at TCA
    pub risk_level: RiskLevel,
}

impl CollisionRisk {
    pub const fn new(debris_id: u8, tca_seconds: u32, miss_distance_km: f32, risk_level: RiskLevel) -> Self {
        Self {
            debris_id,
            tca_seconds,
            miss_distance_km,
            risk_level,
        }
    }
}

/// Maneuver command for collision avoidance
#[derive(Clone, Copy, Debug, Format)]
pub struct ManeuverCommand {
    pub delta_v_mps: f32,        // Magnitude in m/s
    pub direction: Vector3,       // Unit vector
    pub duration_ms: u32,         // Burn duration
}

impl ManeuverCommand {
    pub const fn new(delta_v_mps: f32, direction: Vector3, duration_ms: u32) -> Self {
        Self {
            delta_v_mps,
            direction,
            duration_ms,
        }
    }
}

/// System operational state
#[derive(Clone, Copy, Debug, Format, PartialEq, Eq)]
pub enum SystemState {
    Nominal,
    Monitoring(u8),      // debris_id
    Planning(u8),        // debris_id
    Executing(u8),       // debris_id
    Cooldown,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector3_magnitude() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 0.001);
    }

    #[test]
    fn vector3_dot_product() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(v1.dot(&v2), 32.0);
    }

    #[test]
    fn vector3_cross_product() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let v3 = v1.cross(&v2);
        assert_eq!(v3.x, 0.0);
        assert_eq!(v3.y, 0.0);
        assert_eq!(v3.z, 1.0);
    }

    #[test]
    fn vector3_normalized() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        let n = v.normalized();
        assert!((n.magnitude() - 1.0).abs() < 0.001);
    }
}
