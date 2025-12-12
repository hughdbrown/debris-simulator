//! State machine for maneuver planning and execution

use crate::types::{CollisionRisk, ManeuverCommand, RiskLevel, SystemState};
use crate::collision::calculate_avoidance_maneuver;
use crate::types::OrbitalState;

/// Maneuver planner with safety limits
pub struct ManeuverPlanner {
    pub state: SystemState,
    pub cooldown_remaining_ms: u32,
    pub max_delta_v_mps: f32,
    pub min_cooldown_ms: u32,
}

impl ManeuverPlanner {
    pub const fn new() -> Self {
        Self {
            state: SystemState::Nominal,
            cooldown_remaining_ms: 0,
            max_delta_v_mps: 5.0,           // 5 m/s max delta-v
            min_cooldown_ms: 60_000,        // 60 second cooldown
        }
    }

    /// Update state machine with collision risk
    pub fn update(
        &mut self,
        risk: &CollisionRisk,
        own_ship: &OrbitalState,
        debris_state: &OrbitalState,
    ) -> Option<ManeuverCommand> {
        // Decrement cooldown
        if self.cooldown_remaining_ms > 0 {
            self.cooldown_remaining_ms = self.cooldown_remaining_ms.saturating_sub(200); // Assume 200ms update rate
        }

        match self.state {
            SystemState::Nominal => {
                match risk.risk_level {
                    RiskLevel::High => {
                        if risk.tca_seconds < 300 {
                            // Critical - plan immediately
                            self.state = SystemState::Planning(risk.debris_id);
                        } else {
                            self.state = SystemState::Monitoring(risk.debris_id);
                        }
                        None
                    }
                    RiskLevel::Medium => {
                        self.state = SystemState::Monitoring(risk.debris_id);
                        None
                    }
                    _ => None,
                }
            }

            SystemState::Monitoring(debris_id) => {
                if risk.debris_id != debris_id {
                    // Different debris - re-evaluate
                    self.state = SystemState::Nominal;
                    return self.update(risk, own_ship, debris_state);
                }

                match risk.risk_level {
                    RiskLevel::High if risk.tca_seconds < 300 => {
                        self.state = SystemState::Planning(debris_id);
                        None
                    }
                    RiskLevel::None | RiskLevel::Low => {
                        // Threat passed
                        self.state = SystemState::Nominal;
                        None
                    }
                    _ => None,
                }
            }

            SystemState::Planning(debris_id) => {
                if risk.debris_id != debris_id {
                    self.state = SystemState::Nominal;
                    return self.update(risk, own_ship, debris_state);
                }

                if self.cooldown_remaining_ms > 0 {
                    // Still in cooldown from previous maneuver
                    return None;
                }

                // Calculate avoidance maneuver
                let delta_v_vec = calculate_avoidance_maneuver(
                    own_ship,
                    debris_state,
                    5.0, // 5 km desired miss distance
                );

                let delta_v_mag = delta_v_vec.magnitude();

                if delta_v_mag < 0.01 {
                    // No maneuver needed
                    self.state = SystemState::Nominal;
                    return None;
                }

                // Limit delta-v
                let limited_dv = delta_v_mag.min(self.max_delta_v_mps);
                let direction = delta_v_vec.normalized();

                // Duration: assume 0.1 m/s² thrust
                let duration_ms = ((limited_dv / 0.1) * 1000.0) as u32;

                self.state = SystemState::Executing(debris_id);

                Some(ManeuverCommand::new(limited_dv, direction, duration_ms))
            }

            SystemState::Executing(debris_id) => {
                if risk.debris_id != debris_id {
                    // Different debris - abort current execution (simplified)
                    self.state = SystemState::Nominal;
                    return self.update(risk, own_ship, debris_state);
                }

                // Execution completes externally, then calls finish_execution()
                None
            }

            SystemState::Cooldown => {
                if self.cooldown_remaining_ms == 0 {
                    self.state = SystemState::Nominal;
                }
                None
            }
        }
    }

    /// Call when maneuver execution is complete
    pub fn finish_execution(&mut self) {
        if matches!(self.state, SystemState::Executing(_)) {
            self.state = SystemState::Cooldown;
            self.cooldown_remaining_ms = self.min_cooldown_ms;
        }
    }

    /// Get current state
    pub fn current_state(&self) -> SystemState {
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Vector3;

    #[test]
    fn test_state_machine_transitions() {
        let mut planner = ManeuverPlanner::new();
        assert_eq!(planner.current_state(), SystemState::Nominal);

        // High risk should trigger monitoring
        let risk = CollisionRisk::new(1, 600, 0.5, RiskLevel::High);
        let ship = OrbitalState::new(Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0), 0);
        let debris = OrbitalState::new(Vector3::new(100.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0), 0);

        planner.update(&risk, &ship, &debris);
        assert_eq!(planner.current_state(), SystemState::Monitoring(1));
    }

    #[test]
    fn test_cooldown_enforcement() {
        let mut planner = ManeuverPlanner::new();
        planner.cooldown_remaining_ms = 60000;

        let risk = CollisionRisk::new(1, 100, 0.5, RiskLevel::High);
        let ship = OrbitalState::new(Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0), 0);
        let debris = OrbitalState::new(Vector3::new(100.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0), 0);

        planner.state = SystemState::Planning(1);
        let cmd = planner.update(&risk, &ship, &debris);

        // Should not issue command during cooldown
        assert!(cmd.is_none());
    }
}
