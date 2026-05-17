/*******************************************************************************
 * ACADEMIC CONTEXT:
 *   Institution : Ain Shams University, Faculty of Engineering
 *   Department  : Mechatronics Engineering
 *   Course      : CSE211s – Introduction to Embedded Systems (Spring 2026)
 * 
 * PROJECT DETAILS:
 *   Program     : Autonomous Object-Following Robot Car
 *   Author      : [Team 9 Section 2]
 * 
 * OVERVIEW:
 *   A closed-loop control system that uses an HC-SR04 ultrasonic sensor to 
 *   measure distance in real-time. A PID controller computes the necessary 
 *   motor speeds to maintain a strict target distance from the object ahead.
 * 
 * SYSTEM ARCHITECTURE:
 *   [Hardware]  - STM32 Nucleo Board (mbed-os)
 *               - 2-Wheel Drive Chassis
 *               - L298N Dual H-Bridge Motor Driver
 *               - HC-SR04 Ultrasonic Distance Sensor
 *
 *   [Software]  - Initialization: Pin config, 100Hz PWM, and Timer setups.
 *               - Interrupts (ISRs): 40Hz trigger pulse and echo time-keeping.
 *               - Main Loop: Noise filtering -> PID Calculation -> Motor Drive.
 ******************************************************************************/

#include "mbed.h"

// ============================================================================
// [1] HARDWARE MAPPING & PERIPHERALS
// ============================================================================

// --- Ultrasonic Sensor (HC-SR04) ---
DigitalOut  sonar_trig(D2);    // Sends the 10us activation pulse
InterruptIn sonar_echo(D3);    // Measures the returning pulse width

// --- Motor Controller (L298N) ---
PwmOut      left_pwm(D10);     // Left motor speed control
PwmOut      right_pwm(D11);    // Right motor speed control
DigitalOut  left_fwd(D4);      // Left motor Forward pin
DigitalOut  left_rev(D5);      // Left motor Reverse pin
DigitalOut  right_fwd(D6);     // Right motor Forward pin
DigitalOut  right_rev(D7);     // Right motor Reverse pin

// --- Timing & Interrupt Control ---
Timer       pulse_timer;       // Measures the exact duration of the echo pulse
Ticker      ping_scheduler;    // Triggers the sensor at regular intervals
Timer       loop_timer;        // Tracks elapsed time (dt) for precise PID math

// ============================================================================
// [2] SYSTEM CONFIGURATION & TUNING CONSTANTS
// ============================================================================

struct RobotConfig {
    // Distance targets (in centimeters)
    static constexpr float SETPOINT       = 20.0f; // Target distance to maintain
    static constexpr float STOP_TOLERANCE = 1.5f;  // Deadband (± cm) to stop hunting
    static constexpr float MAX_VALID_DIST = 80.0f; // Ignore readings beyond this

    // Motor Power Limits (0.0 to 1.0)
    static constexpr float MIN_THRUST     = 0.35f; // Min power to overcome friction
    static constexpr float MAX_THRUST     = 0.75f; // Max power (capped to save battery)
    static constexpr float RAMP_RATE      = 0.05f; // Max power change per cycle
};

struct PID_Gains {
    // Controller Tuning
    static constexpr float P_GAIN         = 0.08f;  // Proportional: Direct reaction
    static constexpr float I_GAIN         = 0.002f; // Integral: Overcomes steady errors
    static constexpr float D_GAIN         = 0.025f; // Derivative: Dampens oscillations
    static constexpr float I_LIMIT        = 5.0f;   // Anti-windup cap for Integral term
};

// ============================================================================
// [3] GLOBAL STATE VARIABLES
// ============================================================================

// Volatile variables are used here because they are modified inside Interrupt 
// Service Routines (ISRs) and read in the main loop.
volatile float v_latest_distance = 0.0f;
volatile bool  v_reading_ready   = false;

// ============================================================================
// [4] INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================

/**
 * @brief Triggered when the echo pin goes HIGH (pulse starts).
 */
void onPulseStart() {
    pulse_timer.reset();
    pulse_timer.start();
}

/**
 * @brief Triggered when the echo pin goes LOW (pulse ends).
 *        Calculates the distance based on the speed of sound.
 */
void onPulseEnd() {
    pulse_timer.stop();
    // FIX 1 & 3: Old Mbed OS uses .read_us() instead of .elapsed_time().count()
    long pulse_width_us = pulse_timer.read_us();
    
    // Speed of sound = 0.0343 cm/us. Divide by 2 for round trip.
    v_latest_distance = (pulse_width_us * 0.0343f) / 2.0f;
    v_reading_ready = true;
}

/**
 * @brief Sends a strict 10-microsecond HIGH pulse to the sensor trigger.
 *        Called automatically at 40Hz by the Ticker.
 */
void emitPing() {
    sonar_trig = 1;
    wait_us(10);
    sonar_trig = 0;
}

// ============================================================================
// [5] UTILITY & CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Smooths out sensor noise using a rolling average of 3 samples.
 * @param new_sample The latest raw reading from the sensor.
 * @return The smoothed distance value.
 */
float applyNoiseFilter(float new_sample) {
    static float history[3] = {0};
    static int current_idx = 0;
    static int sample_count = 0;

    // Add new reading to the circular buffer
    history[current_idx] = new_sample;
    current_idx = (current_idx + 1) % 3;
    
    if (sample_count < 3) {
        sample_count++;
    }

    // Calculate the average of available samples
    float accumulator = 0.0f;
    for (int i = 0; i < sample_count; i++) {
        accumulator += history[i];
    }
    return accumulator / sample_count;
}

/**
 * @brief Translates PID output into safe, ramped motor commands.
 *        Handles direction swapping and applies dynamic braking.
 * @param requested_speed Value from -1.0 (full reverse) to +1.0 (full forward).
 */
void updateDrivetrain(float requested_speed) {
    static float active_pwm = 0.0f; // Tracks current applied power

    // Clamp input bounds
    if (requested_speed >  1.0f) requested_speed =  1.0f;
    if (requested_speed < -1.0f) requested_speed = -1.0f;

    // 1. Map requested speed to hardware PWM limits
    float target_pwm = 0.0f;
    if (requested_speed > 0.01f) {
        target_pwm = RobotConfig::MIN_THRUST + (requested_speed * (RobotConfig::MAX_THRUST - RobotConfig::MIN_THRUST));
    } else if (requested_speed < -0.01f) {
        float abs_val = -requested_speed;
        target_pwm = -(RobotConfig::MIN_THRUST + (abs_val * (RobotConfig::MAX_THRUST - RobotConfig::MIN_THRUST)));
    }

    // 2. Slew-rate limiter (Ramping) to prevent voltage drops from high current draw
    float step_diff = target_pwm - active_pwm;
    if (step_diff >  RobotConfig::RAMP_RATE) step_diff =  RobotConfig::RAMP_RATE;
    if (step_diff < -RobotConfig::RAMP_RATE) step_diff = -RobotConfig::RAMP_RATE;
    
    active_pwm += step_diff;

    // Enforce clean zero in the middle
    if (fabsf(active_pwm) < 0.01f) {
        active_pwm = 0.0f;
    }

    float applied_power = fabsf(active_pwm);

    // 3. Shoot-through prevention (Direction Change Safety)
    bool req_forward = (active_pwm > 0.01f);
    bool req_reverse = (active_pwm < -0.01f);

    bool is_forward  = (left_fwd == 1 && left_rev == 0);
    bool is_reverse  = (left_fwd == 0 && left_rev == 1);

    // If reversing direction, cut power entirely for 500us to let transistors close
    if ((req_forward && is_reverse) || (req_reverse && is_forward)) {
        left_pwm.write(0.0f);
        right_pwm.write(0.0f);
        wait_us(500); 
    }

    // 4. Apply Direction and Power
    if (req_forward) {
        left_fwd = 1; left_rev = 0;
        right_fwd = 1; right_rev = 0;
        left_pwm.write(applied_power);
        right_pwm.write(applied_power);
    } 
    else if (req_reverse) {
        left_fwd = 0; left_rev = 1;
        right_fwd = 0; right_rev = 1;
        left_pwm.write(applied_power);
        right_pwm.write(applied_power);
    } 
    else {
        // Active Braking: Shorting motor terminals together
        left_fwd = 1; left_rev = 1;
        right_fwd = 1; right_rev = 1;
        left_pwm.write(0.0f);
        right_pwm.write(0.0f);
        active_pwm = 0.0f; 
    }
}

// ============================================================================
// [6] MAIN EXECUTION LOOP
// ============================================================================

int main() {
    // --- Hardware Initialization ---
    sonar_trig = 0; 

    // Set PWM frequency to 100Hz (10ms period). This allows the older L298N 
    // BJT transistors enough time to fully open/close, reducing heat and stutter.
    left_pwm.period_ms(10);
    right_pwm.period_ms(10);

    // Start safely stopped
    updateDrivetrain(0.0f);

    // --- Interrupt Attachments ---
    sonar_echo.rise(&onPulseStart);
    sonar_echo.fall(&onPulseEnd);
    
    // FIX 2: Ticker setup expects seconds as a float value (25ms = 0.025 seconds)
    ping_scheduler.attach(&emitPing, 0.025f); 

    loop_timer.start();

    // --- Controller States ---
    float accumulated_error = 0.0f;
    float last_error = 0.0f;

    while (true) {
        // Only run math when new hardware data is available
        if (v_reading_ready) {
            v_reading_ready = false; 

            // 1. Process Input Data
            float current_dist = applyNoiseFilter(v_latest_distance);
            if (current_dist > RobotConfig::MAX_VALID_DIST) {
                current_dist = RobotConfig::MAX_VALID_DIST;
            }

            // 2. Calculate Timing (dt)
            // FIX 1 & 3: Read elapsed time as a float representation of seconds directly
            float dt_seconds = loop_timer.read();
            loop_timer.reset();
            if (dt_seconds < 0.001f) dt_seconds = 0.001f; // Prevent div-by-zero

            // 3. Error Calculation
            // Positive error -> target is far -> go forward
            // Negative error -> target is close -> reverse
            float error_val = current_dist - RobotConfig::SETPOINT;

            // 4. Deadband Check
            // If we are close enough, stop completely and reset integrators to prevent windup
            if (fabsf(error_val) <= RobotConfig::STOP_TOLERANCE) {
                accumulated_error = 0.0f;
                last_error = 0.0f;
                updateDrivetrain(0.0f);
                continue; 
            }

            // 5. PID Calculation
            float proportional = PID_Gains::P_GAIN * error_val;

            accumulated_error += error_val * dt_seconds;
            if (accumulated_error >  PID_Gains::I_LIMIT) accumulated_error =  PID_Gains::I_LIMIT;
            if (accumulated_error < -PID_Gains::I_LIMIT) accumulated_error = -PID_Gains::I_LIMIT;
            float integral = PID_Gains::I_GAIN * accumulated_error;

            float error_rate_of_change = (error_val - last_error) / dt_seconds;
            float derivative = PID_Gains::D_GAIN * error_rate_of_change;
            
            last_error = error_val;

            float controller_output = proportional + integral + derivative;

            // 6. Actuate Motors
            updateDrivetrain(controller_output);
        }

        // FIX 4: Changed Modern "ThisThread::sleep_for(2ms)" to universal "wait_ms(2)"
        wait_ms(2);
    }
}