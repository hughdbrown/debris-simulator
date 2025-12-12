#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel as AdcChannel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{ADC, PWM_SLICE0, PWM_SLICE1, PWM_SLICE2};
use embassy_rp::peripherals::{PIN_26, PIN_0, PIN_2, PIN_4};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use debris_simulator::*;

// Inter-task communication channels
static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorReading, 4> = Channel::new();
static RISK_CHANNEL: Channel<CriticalSectionRawMutex, CollisionRisk, 4> = Channel::new();
static MANEUVER_CHANNEL: Channel<CriticalSectionRawMutex, ManeuverCommand, 2> = Channel::new();

// Shared state
static DEBRIS_FIELD: Mutex<CriticalSectionRawMutex, [DebrisObject; 5]> = Mutex::new([
    DebrisObject::new(0, OrbitalState::new(Vector3::ZERO, Vector3::ZERO, 0));
    5
]);

static OWN_SHIP: Mutex<CriticalSectionRawMutex, OrbitalState> = Mutex::new(
    OrbitalState::new(Vector3::ZERO, Vector3::ZERO, 0)
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Debris Avoidance Simulator starting...");

    // Initialize debris field and own ship
    initialize_orbits().await;

    // Spawn all tasks
    if spawner.spawn(sensor_task(p.ADC, p.PIN_26)).is_err() {
        error!("Failed to spawn sensor_task");
    }
    if spawner.spawn(orbit_propagation_task()).is_err() {
        error!("Failed to spawn orbit_propagation_task");
    }
    if spawner.spawn(collision_detection_task()).is_err() {
        error!("Failed to spawn collision_detection_task");
    }
    if spawner.spawn(maneuver_planning_task()).is_err() {
        error!("Failed to spawn maneuver_planning_task");
    }
    if spawner.spawn(actuator_control_task(
        p.PWM_SLICE0, p.PWM_SLICE1, p.PWM_SLICE2,
        p.PIN_0, p.PIN_2, p.PIN_4,
    )).is_err() {
        error!("Failed to spawn actuator_control_task");
    }
    if spawner.spawn(telemetry_task()).is_err() {
        error!("Failed to spawn telemetry_task");
    }

    info!("All tasks spawned. System operational.");

    // Main task just monitors
    loop {
        Timer::after(Duration::from_secs(10)).await;
        info!("System heartbeat");
    }
}

/// Initialize orbital states
async fn initialize_orbits() {
    // Initialize own ship at 500 km circular orbit
    let own_orbit = orbit::circular_orbit(500.0, 0.0, 0.0);
    {
        let mut ship = OWN_SHIP.lock().await;
        *ship = own_orbit;
    }

    // Initialize 5 debris objects at various altitudes
    let debris_altitudes = [450.0, 480.0, 520.0, 550.0, 600.0];
    let mut debris_field = DEBRIS_FIELD.lock().await;

    for (i, altitude) in debris_altitudes.iter().enumerate() {
        let inclination = math::deg_to_rad(i as f32 * 10.0);
        let raan = math::deg_to_rad(i as f32 * 45.0);
        let state = orbit::circular_orbit(*altitude, inclination, raan);
        debris_field[i] = DebrisObject::new(i as u8, state);
    }

    info!("Orbits initialized");
}

/// Task 1: Sensor simulation (10 Hz)
#[embassy_executor::task]
async fn sensor_task(adc_periph: ADC, pin: PIN_26) {
    let mut adc = Adc::new(adc_periph, Irqs, AdcConfig::default());
    let mut channel = AdcChannel::new_pin(pin, embassy_rp::gpio::Pull::None);

    info!("Sensor task started");

    let mut timestamp_ms = 0u64;

    loop {
        // Read ADC (0-4095 for 12-bit ADC)
        let reading: u16 = match adc.read(&mut channel).await {
            Ok(val) => val,
            Err(_) => {
                warn!("ADC read failed, using default value");
                2047  // Midpoint default (500 km)
            }
        };

        // Convert to distance: 0-1000 km
        let distance_km = (reading as f32 / 4095.0) * 1000.0;

        // Simulate azimuth and elevation (would come from actual sensors)
        let azimuth_deg = 0.0;
        let elevation_deg = 0.0;

        let sensor_reading = SensorReading::new(
            distance_km,
            azimuth_deg,
            elevation_deg,
            timestamp_ms,
        );

        // Send to collision detection
        SENSOR_CHANNEL.send(sensor_reading).await;

        timestamp_ms += 100;
        Timer::after(Duration::from_millis(100)).await; // 10 Hz
    }
}

/// Task 2: Orbit propagation (1 Hz)
#[embassy_executor::task]
async fn orbit_propagation_task() {
    info!("Orbit propagation task started");

    loop {
        // Propagate own ship
        {
            let mut ship = OWN_SHIP.lock().await;
            *ship = orbit::propagate_orbit(&*ship, 1.0);
        }

        // Propagate all debris
        {
            let mut debris_field = DEBRIS_FIELD.lock().await;
            for debris in debris_field.iter_mut() {
                debris.state = orbit::propagate_orbit(&debris.state, 1.0);
            }
        }

        Timer::after(Duration::from_secs(1)).await; // 1 Hz
    }
}

/// Task 3: Collision detection (5 Hz)
#[embassy_executor::task]
async fn collision_detection_task() {
    info!("Collision detection task started");

    loop {
        // Get sensor reading (non-blocking try)
        if let Ok(_sensor_reading) = SENSOR_CHANNEL.try_receive() {
            // In a full implementation, would correlate sensor data with tracks
        }

        // Check all debris for collision risks
        let own_ship = {
            let ship = OWN_SHIP.lock().await;
            *ship
        };

        let debris_field = {
            let field = DEBRIS_FIELD.lock().await;
            *field
        };

        // Find highest risk
        let mut highest_risk = CollisionRisk::new(0, u32::MAX, f32::MAX, RiskLevel::None);

        for debris in &debris_field {
            let risk = collision::assess_risk(debris, &own_ship);

            if risk.risk_level as u8 > highest_risk.risk_level as u8 {
                highest_risk = risk;
            }
        }

        // Send to maneuver planner if any risk
        if highest_risk.risk_level != RiskLevel::None {
            info!("Collision risk detected: debris {}, TCA {}s, miss {}km, level {:?}",
                  highest_risk.debris_id, highest_risk.tca_seconds,
                  highest_risk.miss_distance_km, highest_risk.risk_level);
            RISK_CHANNEL.send(highest_risk).await;
        }

        Timer::after(Duration::from_millis(200)).await; // 5 Hz
    }
}

/// Task 4: Maneuver planning state machine
#[embassy_executor::task]
async fn maneuver_planning_task() {
    use state_machine::ManeuverPlanner;

    info!("Maneuver planning task started");

    let mut planner = ManeuverPlanner::new();

    loop {
        // Wait for collision risk
        if let Ok(risk) = RISK_CHANNEL.try_receive() {
            let own_ship = {
                let ship = OWN_SHIP.lock().await;
                *ship
            };

            let debris_state = {
                let field = DEBRIS_FIELD.lock().await;
                field[risk.debris_id as usize].state
            };

            // Update state machine
            if let Some(command) = planner.update(&risk, &own_ship, &debris_state) {
                info!("Maneuver commanded: Δv={}m/s, duration={}ms",
                      command.delta_v_mps, command.duration_ms);
                MANEUVER_CHANNEL.send(command).await;
            }

            info!("State: {:?}", planner.current_state());
        }

        Timer::after(Duration::from_millis(200)).await;
    }
}

/// Task 5: Actuator control (PWM for LEDs)
/// 3 LEDs for X, Y, Z axes (brightness = thrust magnitude)
#[embassy_executor::task]
async fn actuator_control_task(
    slice0: PWM_SLICE0, slice1: PWM_SLICE1, slice2: PWM_SLICE2,
    pin0: PIN_0, pin2: PIN_2, pin4: PIN_4,
) {
    info!("Actuator control task started");

    // Configure PWM for 3 axes (X, Y, Z)
    let mut config = PwmConfig::default();
    config.top = 0xFFFF;
    config.compare_a = 0;
    config.compare_b = 0;

    let mut pwm_x = Pwm::new_output_a(slice0, pin0, config.clone());
    let mut pwm_y = Pwm::new_output_a(slice1, pin2, config.clone());
    let mut pwm_z = Pwm::new_output_a(slice2, pin4, config.clone());

    loop {
        if let Ok(command) = MANEUVER_CHANNEL.try_receive() {
            info!("Executing maneuver...");

            // Map direction vector to thruster activations
            let thrust_x = (command.direction.x * command.delta_v_mps).abs();
            let thrust_y = (command.direction.y * command.delta_v_mps).abs();
            let thrust_z = (command.direction.z * command.delta_v_mps).abs();

            // Convert to PWM duty cycle (0-65535)
            let duty_x = (thrust_x * 6553.5).min(65535.0) as u16;
            let duty_y = (thrust_y * 6553.5).min(65535.0) as u16;
            let duty_z = (thrust_z * 6553.5).min(65535.0) as u16;

            // Set LED brightness based on axis thrust
            config.compare_a = duty_x;
            pwm_x.set_config(&config);

            config.compare_a = duty_y;
            pwm_y.set_config(&config);

            config.compare_a = duty_z;
            pwm_z.set_config(&config);

            // Burn for specified duration
            Timer::after(Duration::from_millis(command.duration_ms as u64)).await;

            // Turn off all LEDs
            config.compare_a = 0;
            pwm_x.set_config(&config);
            pwm_y.set_config(&config);
            pwm_z.set_config(&config);

            info!("Maneuver complete");
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Task 6: Telemetry logging
#[embassy_executor::task]
async fn telemetry_task() {
    info!("Telemetry task started");

    let mut log_counter = 0u32;

    loop {
        let own_ship = {
            let ship = OWN_SHIP.lock().await;
            *ship
        };

        // Log summary every second
        info!("[T={}] Pos: ({}, {}, {}) km, Vel: ({}, {}, {}) km/s",
              log_counter,
              own_ship.position.x, own_ship.position.y, own_ship.position.z,
              own_ship.velocity.x, own_ship.velocity.y, own_ship.velocity.z);

        log_counter += 1;
        Timer::after(Duration::from_secs(1)).await;
    }
}
