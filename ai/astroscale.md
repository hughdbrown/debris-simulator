# Overview
A strong embedded Rust project to submit instead of a resume would simulate orbital debris detection and proximity operations, aligning directly with AstroScale's missions like ELSA-d and ELSA-M for satellite servicing and debris removal. Target a common development board like STM32 or RP2040, using the Embassy crate for async tasks that model satellite rendezvous, sensor polling (e.g., mock distance via ADC), and decision logic for docking avoidance.​​

# Project Concept
Build a "Debris Avoidance Simulator" where an async Embassy executor runs tasks for:
- Periodic "sensor" readings (ADC or timer-driven) simulating LIDAR/radar distances to debris.
- Orbit propagation math (simple numerical integration of Keplerian elements) to predict collision risks.
- A state machine task deciding maneuvers (thrust simulation via PWM output to LEDs/motors).​​

This demonstrates productivity in Embassy's async/await for real-time embedded systems, relevant to spacecraft flight software.​

# Technical Stack
- Framework: Embassy for tasks, channels, and HAL (e.g., embassy-stm32 or embassy-rp).
- Key Crates: embassy-executor, embassy-sync (channels for inter-task comms), embassy-time, HAL-specific crates for peripherals.
- Hardware: RP2040 or STM32 Nucleo (~$20-30); blink LED for "thrust", serial/USB for telemetry output.
- Cargo.toml Snippet (from Embassy book examples):

```text
[dependencies]
embassy-executor = { version = "0.5", features = ["nightly", "integrated-timers"] }
embassy-stm32 = { version = "0.1", features = ["stm32l4r5zx", "time-driver-any"] }
```
Follow Embassy tutorials for setup.

# Mission Tie-In
Link to AstroScale by logging simulated "docking" events inspired by their magnetic capture and multi-debris servicer tech—e.g., output CSV telemetry of avoidance maneuvers. Host on GitHub with README explaining parallels to RPO (rendezvous & proximity operations), Cargo workflow automation (from your CI/CD experience), and a demo video of hardware reacting to "debris" inputs.​

# Detailed Architecture

## System Components

### 1. Sensor Simulation Task
**Purpose**: Simulates LIDAR/radar distance measurements to orbital debris
- Polls ADC (potentiometer) or generates synthetic readings
- Frequency: 10 Hz (100ms intervals)
- Outputs: `SensorReading { distance_km: f32, azimuth_deg: f32, elevation_deg: f32, timestamp_ms: u64 }`
- Uses: `embassy-time::Timer` for periodic execution
- Communication: Sends readings via `embassy_sync::channel::Channel` to collision detector

### 2. Orbit Propagation Task
**Purpose**: Maintains orbital state and predicts future positions
- Uses simplified Two-Body Problem (Keplerian elements)
- Updates: 1 Hz (every second)
- State: `OrbitalState { position: Vector3, velocity: Vector3, epoch_ms: u64 }`
- Algorithm: RK4 numerical integration for position/velocity
- Tracks multiple debris objects (3-5 simulated pieces)
- Communication: Shared state via `embassy_sync::mutex::Mutex` or broadcast channel

### 3. Collision Detection Task
**Purpose**: Analyzes sensor data + predicted orbits to identify collision risks
- Inputs: Sensor readings + orbital predictions
- Frequency: 5 Hz (200ms intervals)
- Algorithm:
  - Calculate Time of Closest Approach (TCA)
  - Compute miss distance at TCA
  - Apply threshold (e.g., < 1km = HIGH risk, < 5km = MEDIUM)
- Output: `CollisionRisk { debris_id: u8, tca_seconds: u32, miss_distance_km: f32, risk_level: RiskLevel }`
- Communication: Sends risk assessments to maneuver planner

### 4. Maneuver Planning State Machine
**Purpose**: Decides avoidance actions based on collision risks
- States: `NOMINAL`, `MONITORING`, `PLANNING`, `EXECUTING`, `COOLDOWN`
- Decision logic:
  - HIGH risk + TCA < 300s → Plan immediate avoidance
  - MEDIUM risk → Monitor closely, prepare contingency
  - LOW risk → Nominal operations
- Outputs: `ManeuverCommand { delta_v_mps: f32, direction: Vector3, duration_ms: u32 }`
- State transitions logged for telemetry

### 5. Actuator Control Task
**Purpose**: Executes maneuver commands via hardware outputs
- PWM output for "thrust" simulation (LED brightness = thrust level)
- Multiple channels: +X, -X, +Y, -Y, +Z, -Z thrusters
- Safety limits: Max delta-v per maneuver, cooldown periods
- Hardware: 6 LEDs or single RGB LED showing dominant axis
- Uses: `embassy_stm32::pwm` or `embassy_rp::pwm`

### 6. Telemetry Task
**Purpose**: Logs all system data for analysis
- Aggregates: Sensor readings, orbital state, collision risks, maneuvers
- Output format: CSV over USB serial or to SD card
- Fields: `timestamp,sensor_distance,debris_id,tca,miss_distance,risk,state,maneuver_dv,actuator_output`
- Frequency: On-demand (triggered by events) or 1 Hz summary
- Uses: `embassy_stm32::usart` or `embassy_rp::uart`

## Data Structures

```rust
// Core types (no_std compatible)
#[derive(Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct OrbitalState {
    pub position: Vector3,      // km
    pub velocity: Vector3,      // km/s
    pub epoch_ms: u64,
}

pub struct DebrisObject {
    pub id: u8,
    pub state: OrbitalState,
}

pub struct SensorReading {
    pub distance_km: f32,
    pub azimuth_deg: f32,
    pub elevation_deg: f32,
    pub timestamp_ms: u64,
}

pub enum RiskLevel {
    None,
    Low,
    Medium,
    High,
}

pub struct CollisionRisk {
    pub debris_id: u8,
    pub tca_seconds: u32,
    pub miss_distance_km: f32,
    pub risk_level: RiskLevel,
}

pub struct ManeuverCommand {
    pub delta_v_mps: f32,
    pub direction: Vector3,
    pub duration_ms: u32,
}

pub enum SystemState {
    Nominal,
    Monitoring(u8),     // debris_id
    Planning(u8),
    Executing(u8),
    Cooldown,
}
```

## Inter-Task Communication

```rust
// Channel definitions
type SensorChannel = Channel<CriticalSectionRawMutex, SensorReading, 4>;
type RiskChannel = Channel<CriticalSectionRawMutex, CollisionRisk, 4>;
type ManeuverChannel = Channel<CriticalSectionRawMutex, ManeuverCommand, 2>;

// Shared state (protected by Mutex)
type OrbitalStateMutex = Mutex<CriticalSectionRawMutex, [DebrisObject; 5]>;
type SystemStateMutex = Mutex<CriticalSectionRawMutex, SystemState>;
```

## Hardware Configuration

### Target: RP2040 (Raspberry Pi Pico)
- **ADC Input**: GP26 for simulated distance sensor
- **PWM Outputs**: GP0-GP5 for 6-axis thruster LEDs
- **UART**: USB serial for telemetry (default USB pins)
- **Clock**: 125 MHz system clock
- **Memory**: 264 KB SRAM (sufficient for no_std + heapless collections)

### Alternative: STM32L4 Nucleo
- **ADC Input**: PA0 (ADC1_IN5)
- **PWM Outputs**: TIM2/TIM3 channels
- **USART**: USART2 (PA2/PA3) for telemetry
- **Clock**: 80 MHz max

## Implementation Plan

### Phase 1: Project Scaffolding (Day 1, 2-3 hours)
1. Create Cargo workspace with `cargo new --lib debris-simulator`
2. Configure `.cargo/config.toml` for target (thumbv6m-none-eabi for RP2040)
3. Set up `Cargo.toml` with Embassy dependencies:
   ```toml
   [dependencies]
   embassy-executor = { version = "0.6", features = ["arch-cortex-m", "executor-thread", "integrated-timers"] }
   embassy-time = { version = "0.3" }
   embassy-rp = { version = "0.2", features = ["time-driver"] }
   embassy-sync = { version = "0.6" }
   cortex-m = { version = "0.7" }
   cortex-m-rt = "0.7"
   defmt = "0.3"
   defmt-rtt = "0.4"
   panic-probe = { version = "0.3", features = ["print-defmt"] }
   heapless = "0.8"
   micromath = "2.0"  # For fast f32 math (sin, cos, sqrt)
   ```
4. Create `memory.x` linker script for RP2040
5. Verify build: `cargo build --release`
6. Flash and run simple LED blink to confirm toolchain

### Phase 2: Core Data Structures & Math (Day 1, 3-4 hours)
1. Implement `Vector3` with basic ops (add, sub, scale, dot, cross, magnitude)
2. Implement orbital propagator:
   - Gravitational parameter μ = 398600.4418 km³/s²
   - RK4 integrator for d/dt(r) = v, d/dt(v) = -μr/|r|³
   - Step size: 1 second
3. Unit tests for orbit propagation (ground truth from published data)
4. Initialize debris field with randomized but realistic LEO orbits (400-800 km altitude)

### Phase 3: Sensor & Orbit Tasks (Day 1-2, 4 hours)
1. Implement sensor task:
   - Read ADC (0-3.3V → 0-1000 km distance)
   - Add Gaussian noise for realism
   - Publish to channel
2. Implement orbit propagation task:
   - Maintain array of 5 `DebrisObject`
   - Update states every 1s
   - Store in shared Mutex
3. Debug output via defmt to verify task scheduling

### Phase 4: Collision Detection (Day 2, 3-4 hours)
1. Implement TCA calculation:
   - Relative position/velocity between "own ship" and debris
   - Solve for minimum |r_rel(t)| using calculus
2. Implement risk assessment logic
3. Subscribe to sensor + orbital channels
4. Publish `CollisionRisk` messages
5. Test with synthetic close approach scenarios

### Phase 5: Maneuver State Machine (Day 2, 3-4 hours)
1. Implement state machine with Rust enums
2. Decision logic based on risk level + TCA
3. Calculate delta-v using simplified impulsive maneuver equations
4. Safety checks: max delta-v limits, cooldown enforcement
5. Publish `ManeuverCommand` to actuator task

### Phase 6: Actuator Control (Day 2, 2 hours)
1. Configure 6 PWM channels (or map to RGB LED)
2. Map delta-v vector to thruster activation
3. Execute timed burns (use Timer::after for duration)
4. Visual feedback: LED brightness proportional to thrust

### Phase 7: Telemetry (Day 2-3, 2-3 hours)
1. Configure UART/USB serial
2. Implement CSV formatter (heapless::String for no_std)
3. Aggregate data from all tasks (use Signal or PubSubChannel)
4. Stream at 1 Hz or on event triggers
5. Test with serial terminal capture

### Phase 8: Integration & Polish (Day 3, 3-4 hours)
1. End-to-end test: Turn ADC pot → sensor reading → collision detected → LEDs activate
2. Record demo video showing:
   - Nominal operations (slow ADC movement)
   - Close approach detection (fast ADC change)
   - Maneuver execution (LEDs lighting up)
   - Telemetry dump in terminal
3. Add README.md explaining:
   - Hardware setup (pinout diagram)
   - How to build/flash
   - Connection to AstroScale missions
4. GitHub Actions for CI (cargo fmt, clippy, build check)
5. Optional: 1-page PDF linking skills to job requirements

## Testing Strategy

### Unit Tests (Desktop)
- Vector math operations (cross product, dot product)
- Orbit propagation accuracy (compare to analytical solutions)
- TCA calculation with known close approaches
- State machine transitions

### Hardware-in-Loop Tests
- Mock sensor inputs via voltage divider
- Verify PWM output with oscilloscope/multimeter
- Serial output parsing and validation
- Task timing analysis (measure jitter with logic analyzer)

### Simulation Tests
- Run full system with synthetic debris field
- Inject collision scenarios at T-300s, T-60s, T-10s
- Verify correct state transitions and maneuver commands
- Log telemetry for offline analysis

## AstroScale Mission Parallels

| Simulator Feature | AstroScale Mission Element |
|------------------|---------------------------|
| Sensor simulation (LIDAR) | ELSA-d rendezvous sensors |
| Orbit propagation | On-board navigation & GNC |
| Collision detection | Proximity operations safety |
| Maneuver planning | Automated RPO sequencing |
| Actuator control | Thruster management system |
| Telemetry logging | Ground station data downlink |
| State machine | ELSA-M multi-debris sequencing |

## Extensions (If Time Permits)

1. **Kalman Filter**: Fuse sensor readings with orbit predictions
2. **Fuel Budget**: Track propellant usage, enforce mission limits
3. **Multi-Debris Prioritization**: Handle simultaneous threats
4. **HITL Mode**: Accept manual commands via serial (simulate ground override)
5. **Flash Storage**: Log telemetry to SPI flash for later retrieval
6. **3D Visualization**: Output telemetry to companion Python script for plotting

# Submission Tips
Push clean, documented code with cargo fmt and tests; include a 1-page PDF tying features to job reqs (e.g., "Async tasks mirror satellite control loops"). Your Rust background (LeetCode, Yew/Axum) plus aerospace interest positions this as authentic. This beats a resume by proving hands-on skills in 1-2 days of work.
