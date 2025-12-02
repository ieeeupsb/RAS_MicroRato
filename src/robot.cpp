/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "robot.h"
#include "IRLine.h"
#include "hardware_config.h"

/**
 * @file robot.cpp
 * @brief Implementation of robot control functions
 * 
 * This file implements motor control, line following with PID,
 * and navigation commands (turns, forward, reverse).
 */

// Global robot instance
robot_t robot;

// Speed constants (from hardware_config.h)
int nominal_speed = NOMINAL_SPEED;  // 80 - for turns and maneuvers
int follow_speed = FOLLOW_SPEED;    // 50 - for line following base speed

/**
 * @brief Constructor - Initialize robot parameters
 * 
 * Sets up physical robot dimensions and control parameters:
 * - Wheel geometry for odometry calculations
 * - Maximum acceleration limits
 * - Control loop timing
 * - Line following gains
 */
robot_t::robot_t()
{
  // Physical parameters
  wheel_dist = 0.125;         // Distance between wheels (meters)
  wheel_radius = 0.0689 / 2;  // Wheel radius (meters), diameter = 68.9mm
  
  // Acceleration limits (for future use with velocity control)
  dv_max = 5;   // Maximum linear acceleration (m/s²)
  dw_max = 10;  // Maximum angular acceleration (rad/s²)
  
  // Control loop timing
  dt = 0.04;    // 40ms = 25Hz control loop (not currently enforced)

  // Line following parameters (alternative method, unused)
  follow_k = -0.15;  // Proportional gain
  follow_v = 0.20;   // Forward velocity (m/s)
}


// ============================================================================
// ODOMETRY (Defined but unused)
// ============================================================================

/**
 * @brief Update robot pose estimate using encoder data
 * 
 * This implements differential drive odometry:
 * 1. Convert encoder ticks to wheel velocities
 * 2. Calculate robot linear and angular velocities
 * 3. Integrate velocities to estimate position and heading
 * 
 * Differential drive kinematics:
 * - v = (v_left + v_right) / 2  (linear velocity)
 * - w = (v_right - v_left) / L  (angular velocity)
 * 
 * Where:
 * - v_left, v_right: Left and right wheel velocities
 * - L: Distance between wheels
 * 
 * @note Currently not called from main loop
 * @note Requires encoder initialization with init_PIO_dual_encoders()
 * @todo Integrate this for accurate position tracking
 */
void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  // 1920 ticks per revolution (encoder resolution)
  // TWO_PI / (2.0 * 1920.0) = radians per tick
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  // Convert angular velocity to linear velocity
  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot velocities (differential drive)
  ve = (v1e + v2e) / 2.0;            // Linear velocity (average)
  we = (v1e - v2e) / wheel_dist;     // Angular velocity (difference / baseline)
  
  // Estimate the distance and the turn angle this cycle
  ds = ve * dt;       // Linear displacement
  dtheta = we * dt;   // Angular displacement

  // Estimate pose using midpoint integration
  // This assumes constant curvature during the interval
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Track relative displacement (for turn completion detection)
  rel_s += ds;
  rel_theta += dtheta;
}


// ============================================================================
// VELOCITY CONTROL (Unused)
// ============================================================================

/**
 * @brief Apply acceleration limits to velocity commands
 * 
 * Implements smooth ramping of velocities to prevent:
 * - Wheel slip from sudden acceleration
 * - Mechanical stress on drivetrain
 * - Encoder counting errors from rapid changes
 * 
 * Algorithm:
 * 1. Calculate velocity change needed
 * 2. Clamp to maximum acceleration
 * 3. Apply limited change to current velocity
 * 
 * @note Currently not used - direct PWM control has no ramping
 * @todo Call this in VWToMotorsVoltage() for smoother motion
 */
void robot_t::accelerationLimit(void)
{
  // Limit linear velocity change
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  // Limit angular velocity change
  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


/**
 * @brief Convert V,W commands to motor voltages
 * 
 * Implements two control modes:
 * 1. cm_pwm: Direct PWM pass-through (currently used)
 * 2. cm_pid: Closed-loop velocity control (not implemented)
 * 
 * Differential drive inverse kinematics:
 * - v1 = v + w * L / 2  (left wheel velocity)
 * - v2 = v - w * L / 2  (right wheel velocity)
 * 
 * Where:
 * - v: Robot linear velocity
 * - w: Robot angular velocity
 * - L: Wheel baseline (distance between wheels)
 * 
 * @note PID mode is commented out and incomplete
 * @note Currently only cm_pwm mode is functional
 */
void robot_t::VWToMotorsVoltage(void)
{
  // Calculate wheel velocity references from robot velocity
  v1ref = v + w * wheel_dist / 2;  // Left wheel
  v2ref = v - w * wheel_dist / 2;  // Right wheel
  
  // Convert to angular velocities
  w1ref = v1ref * wheel_radius;
  w2ref = v2ref * wheel_radius;

  // Direct PWM mode: Use requested PWM values directly
  if (control_mode == cm_pwm) {
    PWM_1 = PWM_1_req;  
    PWM_2 = PWM_2_req;  
  }

  // PID velocity control mode (incomplete, commented out)
  // This would:
  // 1. Compare reference velocity to measured velocity
  // 2. Use PID to calculate control voltage
  // 3. Convert voltage to PWM based on battery voltage
  //
  // else if (control_mode == cm_pid) {
  //   u1 = 0;
  //   u2 = 0;      
  //
  //   if (v1ref != 0) u1 = PID[0].calc(v1ref, v1e);
  //   else PID[0].Se = 0;
  //
  //   if (v2ref != 0) u2 = PID[1].calc(v2ref, v2e);
  //   else PID[1].Se = 0;
  //
  //   PWM_1 = u1 / battery_voltage * 255;
  //   PWM_2 = u2 / battery_voltage * 255;
  // }
}


// ============================================================================
// MOTOR CONTROL
// ============================================================================

/**
 * @brief Apply PWM signal to a motor through H-bridge driver
 * 
 * H-Bridge Control Logic (Sign-Magnitude PWM):
 * +----------+----------+----------+-------------------+
 * | new_PWM  | pin_a    | pin_b    | Motor Action      |
 * +----------+----------+----------+-------------------+
 * | > 0      | 255-PWM  | 255      | Forward (A low)   |
 * | < 0      | 255      | 255+PWM  | Reverse (B low)   |
 * | = 0      | 255      | 255      | Brake (both high) |
 * +----------+----------+----------+-------------------+
 * 
 * Why 255 - PWM?
 * - H-bridge responds to low pulses (active low)
 * - PWM 200 → pin gets 255-200 = 55 → 78% duty low → 78% power
 * - This inverts the PWM so higher values = more power
 * 
 * Why both HIGH for stop?
 * - Both MOSFETs ON → motor terminals shorted → dynamic braking
 * - Alternative (both LOW): motor terminals floating → coast to stop
 * - Braking provides better position control and faster stops
 * 
 * @param new_PWM Motor power (-255 to +255)
 *                Positive = forward, Negative = reverse, Zero = brake
 * @param pin_a H-bridge input A (typically IN1 or AIN1)
 * @param pin_b H-bridge input B (typically IN2 or AIN2)
 * 
 * @note PWM is clamped to ±200 (not ±255) to protect motors/drivers
 * @note Consider making PWM_max configurable in hardware_config.h
 */
void robot_t::setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  // Clamp PWM to safe limits
  int PWM_max = 200;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  
  if (new_PWM == 0) 
  {  
    // Stop: Both outputs HIGH → Short brake
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255);
  } 
  else if (new_PWM > 0) 
  {
    // Forward: pin_a LOW (with PWM), pin_b HIGH
    analogWrite(pin_a, 255 - new_PWM);  // Inverted PWM
    analogWrite(pin_b, 255);            // Full high
  } 
  else 
  {
    // Reverse: pin_a HIGH, pin_b LOW (with PWM)
    analogWrite(pin_a, 255);            // Full high
    analogWrite(pin_b, 255 + new_PWM);  // Inverted PWM (new_PWM is negative)
  }
}


// ============================================================================
// SENSOR HELPER
// ============================================================================

/**
 * @brief Calculate sum of all IR sensor readings
 * 
 * Used to detect overall line presence:
 * - Sum < 2500: Robot is on a line (normal line following)
 * - Sum ≥ 2500: Robot might be at junction or off line
 * 
 * Why 2500 threshold?
 * - 5 sensors × 500 average = 2500
 * - Junction has more black → higher sum
 * - All white has low sum
 * 
 * @return int Sum of all 5 IR sensor values (0-5115 range)
 * @note Threshold may need tuning for different line widths/colors
 */
int robot_t::IR_sum()
{
  return IRLine.IR_values[0] + IRLine.IR_values[1] + 
         IRLine.IR_values[2] + IRLine.IR_values[3] + 
         IRLine.IR_values[4];
}


// ============================================================================
// LINE FOLLOWING
// ============================================================================

/**
 * @brief PID-based line following
 * 
 * This is the main control algorithm for following the line:
 * 
 * 1. Error Calculation:
 *    Uses weighted sum of outer sensors to determine line position
 *    error = -1.1*IR[0] - 1.0*IR[1] + 1.0*IR[3] + 1.1*IR[4]
 *    
 *    - Negative error: Line is to the LEFT → turn left
 *    - Positive error: Line is to the RIGHT → turn right
 *    - Outer sensors weighted more (1.1) for faster response
 * 
 * 2. PID Calculation:
 *    correction = Kp*error + Ki*integral + Kd*derivative
 *    
 *    - Proportional: Responds to current error
 *    - Integral: Eliminates steady-state error (currently disabled, Ki=0)
 *    - Derivative: Dampens oscillations, improves stability
 * 
 * 3. Motor Control:
 *    - Base speed: FOLLOW_SPEED (50)
 *    - Left motor: base + correction
 *    - Right motor: base - correction
 *    
 *    Effect: If line is right, slow right motor and speed up left motor
 * 
 * 4. Safety:
 *    - Correction clamped to ±100 to prevent extreme speed differences
 *    - Stop if all sensors see white (lost line)
 * 
 * Tuning Notes:
 * - Kp=0.12, Kd=0.32 works well (noted in comments)
 * - Increasing Kd reduces wobble but may slow response
 * - Ki not needed for this application (steady-state error acceptable)
 * 
 * @note Called continuously from FOLLOW_LINE states
 * @note IR_sum threshold (2500) may need adjustment
 */
void robot_t::followLine() 
{
  int IR_tresh = robot.IRLine.IR_tresh;
  
  // Check if we're on a line (not at junction or lost)
  if (robot.IR_sum() < 2500) 
  {
    // Calculate error: weighted position from sensors
    // Outer sensors (0,4) weighted higher for sensitivity
    error = -1.1 * IRLine.IR_values[0] - 1.0 * IRLine.IR_values[1] + 
             1.0 * IRLine.IR_values[3] + 1.1 * IRLine.IR_values[4];
    
    Serial.printf(" %d ", error);

    // PID calculations
    integral += error;                  // Accumulate error (Ki=0, so unused)
    derivative = error - prevError;     // Rate of change
    prevError = error;                  // Store for next iteration

    // Compute correction using PID formula
    int correction = (IRkp * error + IRki * integral + IRkd * derivative);

    // Clamp correction to reasonable limits
    if (correction > 100) correction = 100;
    else if (correction < -100) correction = -100;

    // Apply differential correction to motors
    // If error > 0 (line to right): slow right, speed left
    // If error < 0 (line to left): slow left, speed right
    int right_PWM = follow_speed - correction;
    int left_PWM = follow_speed + correction;

    // Update motor commands
    PWM_1 = left_PWM;
    PWM_2 = right_PWM;
  }
  else 
  {
    // At junction or off line: reset integral and go straight
    integral = 0;
    PWM_1 = follow_speed;
    PWM_2 = follow_speed;
  }
  
  // Safety: Stop if completely off line (all white)
  if (IRLine.IR_values[0] < IR_tresh && 
      IRLine.IR_values[1] < IR_tresh && 
      IRLine.IR_values[2] < IR_tresh && 
      IRLine.IR_values[3] < IR_tresh && 
      IRLine.IR_values[4] < IR_tresh)
  {
    stop(); 
  }
}

// Commented out alignment function from original code
// This appears to be an attempt at precise junction alignment using PID
// but was left incomplete
//
// bool robot_t::align() 
// {
//   int IR_tresh = robot.IRLine.IR_tresh;
//   
//   error = -1.1 * IRLine.IR_values[0] - 1.0 * IRLine.IR_values[1] + 
//            1.0 * IRLine.IR_values[3] + 1.1 * IRLine.IR_values[4];
//
//   if(error < 400 ) {
//     return true;  // Aligned within threshold
//   }
//
//   integral += error;
//   derivative = error - prevError;
//   prevError = error;
//
//   int correction = (IRkp * error + IRki * integral + IRkd * derivative);
//
//   if (correction > 100) correction = 100;
//   else if (correction < -100) correction = -100;
//
//   // Apply correction as rotation in place
//   int right_PWM = correction;
//   int left_PWM = -correction;
//
//   PWM_1 = left_PWM;
//   PWM_2 = right_PWM;
// }


// ============================================================================
// NAVIGATION COMMANDS
// ============================================================================

/**
 * @brief Stop the robot
 * 
 * Sets both motors to zero PWM, which activates dynamic braking
 * (both H-bridge outputs HIGH, shorting motor terminals).
 */
void robot_t::stop()
{
  robot.PWM_1 = 0;
  robot.PWM_2 = 0; 
}

/**
 * @brief Execute a 90° right turn
 * 
 * Time-based turn using differential drive:
 * - Left motor: NOMINAL_SPEED (forward)
 * - Right motor: 0 (stopped)
 * - Duration: 800ms
 * 
 * This creates rotation about the right wheel contact point.
 * 
 * Implementation:
 * - Uses static variable to remember start time across calls
 * - First call: start timer and apply motor commands
 * - Subsequent calls: check if time elapsed
 * - When complete: set END_TURN flag and reset timer
 * 
 * @note 800ms is empirically determined - may need calibration
 * @note Turn accuracy depends on battery voltage, surface, and wheel wear
 * @todo Replace with encoder-based turn for precision
 */
void robot_t::right_turn()
{
  static unsigned long start_time = 0;
  
  if (start_time == 0) {
    // First call: start the turn
    start_time = millis();
    PWM_1 = nominal_speed;  // Left motor forward
    PWM_2 = 0;              // Right motor stopped
  }

  if (millis() - start_time > 800) {  // 800ms for ~90°
    END_TURN = true;
    start_time = 0;  // Reset for next turn
  }
}

/**
 * @brief Execute a 90° left turn
 * 
 * Time-based turn using differential drive:
 * - Left motor: 0 (stopped)
 * - Right motor: NOMINAL_SPEED (forward)
 * - Duration: 800ms
 * 
 * Mirror image of right_turn().
 * 
 * @note Same limitations as right_turn()
 */
void robot_t::left_turn()
{
  static unsigned long start_time = 0;
  
  if (start_time == 0) {
    // First call: start the turn
    start_time = millis();
    PWM_1 = 0;              // Left motor stopped
    PWM_2 = nominal_speed;  // Right motor forward
  }

  if (millis() - start_time > 800) {  // 800ms for ~90°
    END_TURN = true;
    start_time = 0;  // Reset for next turn
  }
}

/**
 * @brief Execute a 180° U-turn
 * 
 * Time-based turn using counter-rotation:
 * - Left motor: NOMINAL_SPEED (forward)
 * - Right motor: -NOMINAL_SPEED (backward)
 * - Duration: 80ms
 * 
 * This spins the robot in place about its center.
 * 
 * @note 80ms seems VERY short for 180° turn
 *       At 800ms for 90°, expect ~1600ms for 180°
 *       This may be a bug - verify actual turn angle
 * @todo Measure actual turn angle and adjust timing
 * @todo Consider making turn times constants in hardware_config.h
 */
void robot_t::u_turn()
{
  static unsigned long start_time = 0;
  
  if (start_time == 0) {
    // First call: start the turn
    start_time = millis();
    PWM_1 = nominal_speed;   // Left motor forward
    PWM_2 = -nominal_speed;  // Right motor backward
  }

  if (millis() - start_time > 80) {  // WARNING: 80ms seems too short!
    END_TURN = true;
    start_time = 0;  // Reset for next turn
  }
}

/**
 * @brief Move backward at nominal speed
 * 
 * Both motors reverse at NOMINAL_SPEED.
 * Used to back away from dead ends or bad junctions.
 * 
 * @note No timeout - caller must transition out of this state
 */
void robot_t::reverse()
{
  PWM_1 = -nominal_speed;
  PWM_2 = -nominal_speed;
}

/**
 * @brief Move forward at nominal speed
 * 
 * Both motors forward at NOMINAL_SPEED.
 * Used for crossing junctions or short straight movements.
 * 
 * @note No timeout - caller must transition out of this state
 */
void robot_t::forward()
{
  PWM_1 = nominal_speed;
  PWM_2 = nominal_speed;
}