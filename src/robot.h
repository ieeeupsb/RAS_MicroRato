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

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <math.h>
//#include "PID.h"  // NOTE: PID class not used, using manual PID implementation
#include "IRLine.h"
#include "state_machines.h"

/** @brief Number of drive wheels (2 for differential drive) */
#ifndef NUM_WHEELS
#define NUM_WHEELS 2
#endif

/**
 * @brief Motor control modes
 * 
 * - cm_pwm: Direct PWM control (currently used)
 * - cm_pid: Closed-loop velocity control (not implemented)
 */
typedef enum { 
  cm_pwm,  ///< Direct PWM motor control (0-255)
  cm_pid   ///< PID velocity control (requires encoders)
} control_mode_t;

/**
 * @file robot.h
 * @brief Robot class definition for motor control, navigation, and line following
 * 
 * This class encapsulates all robot behavior including:
 * - Motor control (PWM-based)
 * - Line following with PID control
 * - Navigation commands (turns, forward, reverse)
 * - Odometry (defined but not implemented)
 * - Sensor interface (IR line sensors)
 */
class robot_t {
  public:

    // ========================================================================
    // ENCODER & ODOMETRY (Currently unused)
    // ========================================================================
    
    /** @brief Encoder tick counts since last reset */
    int enc1, enc2;
    
    /** @brief Estimated wheel angular velocities (rad/s) */
    float w1e, w2e;
    
    /** @brief Estimated wheel linear velocities (m/s) */
    float v1e, v2e;
    
    /** @brief Estimated robot velocities (linear m/s, angular rad/s) */
    float ve, we;
    
    /** @brief Odometry increments per cycle */
    float ds, dtheta;
    
    /** @brief Relative displacement since last reset */
    float rel_s, rel_theta;
    
    /** @brief Estimated robot pose (x, y in meters, theta in radians) */
    float xe, ye, thetae;

    // ========================================================================
    // TURN CONTROL
    // ========================================================================
    
    /** 
     * @brief Flag indicating turn completion
     * 
     * Set by turn functions (left_turn, right_turn, u_turn) when the
     * time-based turn is complete. State machine checks this to know
     * when to transition back to line following.
     */
    bool END_TURN = false;
    
    /** @brief Timestamp when current turn started (milliseconds) */
    unsigned long turn_start_time = 0;
    
    /** @brief Flag indicating robot is currently turning */
    bool is_turning = false;

    // ========================================================================
    // VELOCITY CONTROL (Unused - for future encoder-based control)
    // ========================================================================
    
    /** @brief Control loop time step (seconds) */
    float dt;
    
    /** @brief Current linear and angular velocities (m/s, rad/s) */
    float v, w;
    
    /** @brief Requested velocities (m/s, rad/s) */
    float v_req, w_req;
    
    /** @brief Maximum acceleration limits (m/s², rad/s²) */
    float dv_max, dw_max;

    // ========================================================================
    // ROBOT PHYSICAL PARAMETERS
    // ========================================================================
    
    /** @brief Wheel radius (meters) - used for odometry */
    float wheel_radius;
    
    /** @brief Distance between wheels (meters) - used for odometry */
    float wheel_dist;

    // ========================================================================
    // MOTOR REFERENCE VALUES (Unused - for PID velocity control)
    // ========================================================================
    
    /** @brief Reference linear velocities for each wheel (m/s) */
    float v1ref, v2ref;
    
    /** @brief Reference angular velocities for each wheel (rad/s) */
    float w1ref, w2ref;
    
    /** @brief Control signals (voltage) for each wheel */
    float u1, u2;

    // ========================================================================
    // MOTOR PWM OUTPUTS
    // ========================================================================
    
    /** 
     * @brief Current PWM values for motors (-255 to +255)
     * 
     * - Positive: Forward
     * - Negative: Backward
     * - Zero: Stop (both motor pins HIGH for braking)
     * 
     * Applied to motors via setMotorPWM() function
     */
    int PWM_1, PWM_2;
    
    /** @brief Requested PWM values (for cm_pwm mode) */
    int PWM_1_req, PWM_2_req;
    
    /** @brief Current motor control mode */
    control_mode_t control_mode;

    // ========================================================================
    // PID PARAMETERS FOR LINE FOLLOWING
    // ========================================================================
    
    /**
     * @brief PID gains for line following
     * 
     * Tuned values:
     * - Kp = 0.12: Proportional gain (responds to current error)
     * - Ki = 0.0:  Integral gain (eliminates steady-state error, disabled)
     * - Kd = 0.32: Derivative gain (dampens oscillations)
     * 
     * Good performance noted in comments with Kd=0.35 and Kp=0.12
     */
    double IRkp = 0.12, IRki = 0, IRkd = 0.32;
    
    /** @brief Previous PID values (unused - perhaps for tuning UI) */
    double lastIRkp = IRkp, lastIRki = IRki, lastIRkd = IRkd;

    /**
     * @brief PID state variables
     * 
     * - error: Current line position error
     * - prevError: Previous error for derivative calculation
     * - integral: Accumulated error for integral term (currently Ki=0)
     * - derivative: Rate of error change
     */
    double error;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;

    // ========================================================================
    // UNUSED VELOCITY VARIABLES
    // ========================================================================
    
    /** @brief Wheel velocity components (unused) */
    float right_v = 0.0, left_v = 0.0, right_w = 0.0, left_w = 0.0;

    // ========================================================================
    // LINE FOLLOWING PARAMETERS (Unused - for VW control mode)
    // ========================================================================
    
    /** @brief Nominal forward velocity for line following (m/s) */
    float follow_v;
    
    /** @brief Proportional gain for line following (alternative method) */
    float follow_k;

    // ========================================================================
    // SYSTEM STATE
    // ========================================================================
    
    //PID_t PID[NUM_WHEELS];  // NOTE: Unused PID class instances
    
    /** @brief Battery voltage (for voltage compensation in PID mode) */
    float battery_voltage;
    
    /** @brief User button state (unused, handled in button_handler) */
    int button_state;
    
    /** @brief IR line sensor array interface */
    IRLine_t IRLine;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    
    /**
     * @brief Construct robot object and initialize parameters
     * 
     * Sets physical parameters and default control gains.
     * See implementation for details.
     */
    robot_t();

    // ========================================================================
    // ODOMETRY FUNCTIONS (Defined but unused)
    // ========================================================================
    
    /**
     * @brief Update robot pose estimate using encoder readings
     * 
     * @note Currently not called from main loop
     * @note Requires encoders to be initialized
     * @todo Integrate with encoder readings for accurate navigation
     */
    void odometry(void);

    // ========================================================================
    // VELOCITY CONTROL FUNCTIONS (Unused)
    // ========================================================================
    
    /**
     * @brief Set robot velocity commands (linear and angular)
     * @param Vnom Linear velocity (m/s)
     * @param Wnom Angular velocity (rad/s)
     * @note Function declared but not defined
     */
    void setRobotVW(float Vnom, float Wnom);
    
    /**
     * @brief Apply acceleration limits to velocity commands
     * @note Currently not used
     * @todo Use this for smoother motion control
     */
    void accelerationLimit(void);
    
    /**
     * @brief Convert V,W commands to motor voltages
     * @note Currently not used (using direct PWM instead)
     * @todo Implement for cm_pid control mode
     */
    void VWToMotorsVoltage(void);

    // ========================================================================
    // MOTOR CONTROL
    // ========================================================================
    
    /**
     * @brief Apply PWM signal to a motor
     * 
     * Controls H-bridge driver with sign-magnitude PWM:
     * - PWM > 0: Forward (pin_a LOW, pin_b HIGH with PWM)
     * - PWM < 0: Reverse (pin_a HIGH with PWM, pin_b LOW)
     * - PWM = 0: Brake (both pins HIGH)
     * 
     * @param new_PWM PWM value (-255 to +255)
     * @param pin_a H-bridge input A
     * @param pin_b H-bridge input B
     * 
     * @note PWM is clamped to ±200 (not full ±255)
     * @note Both pins HIGH = short brake, both LOW = coast
     */
    void setMotorPWM(int new_PWM, int pin_a, int pin_b);

    // ========================================================================
    // LINE FOLLOWING FUNCTIONS
    // ========================================================================
    
    /**
     * @brief Alternative line following (right edge, unused)
     * @param Vnom Nominal velocity
     * @param K Proportional gain
     * @note Not called in current implementation
     */
    void followLineRight(float Vnom, float K);
    
    /**
     * @brief Alternative line following (left edge, unused)
     * @param Vnom Nominal velocity
     * @param K Proportional gain
     * @note Not called in current implementation
     */
    void followLineLeft(float Vnom, float K);
    
    /**
     * @brief Main PID-based line following function
     * 
     * Algorithm:
     * 1. Calculate weighted error from sensors
     * 2. Compute PID correction
     * 3. Apply differential speed to motors
     * 4. Stop if all sensors see white
     * 
     * Error calculation:
     * error = -1.1*IR[0] - 1.0*IR[1] + 1.0*IR[3] + 1.1*IR[4]
     * 
     * - Negative: Line is to the left
     * - Positive: Line is to the right
     * 
     * Called continuously from FOLLOW_LINE states in state machines.
     */
    void followLine();

    // ========================================================================
    // NAVIGATION COMMANDS
    // ========================================================================
    
    /**
     * @brief Stop the robot (set both motors to zero)
     */
    void stop();
    
    /**
     * @brief Execute a 90° left turn
     * 
     * Timing-based turn:
     * - Left motor: 0 PWM
     * - Right motor: NOMINAL_SPEED PWM
     * - Duration: 800ms
     * 
     * Sets END_TURN flag when complete.
     * 
     * @note Time-based, may need calibration for accuracy
     * @todo Replace with encoder-based turn for precision
     */
    void left_turn();
    
    /**
     * @brief Execute a 90° right turn
     * 
     * Timing-based turn:
     * - Left motor: NOMINAL_SPEED PWM
     * - Right motor: 0 PWM
     * - Duration: 800ms
     * 
     * @note Mirror of left_turn()
     */
    void right_turn();
    
    /**
     * @brief Execute a 180° U-turn
     * 
     * Timing-based turn:
     * - Left motor: NOMINAL_SPEED PWM (forward)
     * - Right motor: -NOMINAL_SPEED PWM (backward)
     * - Duration: 80ms
     * 
     * @note 80ms seems very short for 180° - may need verification
     * @note Using differential drive (opposite wheel directions)
     */
    void u_turn();
    
    /**
     * @brief Move backward at nominal speed
     * 
     * Both motors reverse at NOMINAL_SPEED.
     * Used to back away from dead ends or bad junctions.
     */
    void reverse();
    
    /**
     * @brief Move forward at nominal speed
     * 
     * Both motors forward at NOMINAL_SPEED.
     * Used for crossing junctions or short straight movements.
     */
    void forward();

    // ========================================================================
    // SENSOR HELPER
    // ========================================================================
    
    /**
     * @brief Calculate sum of all IR sensor readings
     * 
     * Used to detect if robot is on/off the line:
     * - Low sum (< 2500): On line
     * - High sum: Off line or at junction
     * 
     * @return int Sum of all 5 sensor values
     */
    int IR_sum();
};

/**
 * @brief Global robot instance
 * 
 * Single global instance of the robot, accessible from all modules.
 * Defined in robot.cpp, declared extern here.
 */
extern robot_t robot;

#endif // ROBOT_H