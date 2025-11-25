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
#include "config.h"

/**
 * @file robot.cpp
 * @brief Robot low-level control implementation for Micro-Rato-2025.
 *
 * This file contains the implementation of the robot_t methods used by the
 * state machines and main loop to move the platform, follow the line and
 * convert requested linear/angular velocities to motor PWM values.
 *
 */

/**
 * Global robot instance.
 */
robot_t robot;

/**
 * Nominal forward speed used by simple motion primitives (PWM units).
 * Value comes from `config.h` as `NOMINAL_SPEED`.
 */
int nominal_speed = NOMINAL_SPEED;

/**
 * Speed used while following the line (PWM units).
 * Value comes from `config.h` as `FOLLOW_SPEED`.
 */
int follow_speed = FOLLOW_SPEED;

/**
 * @brief Construct a new robot_t object and initialize robot parameters.
 *
 * Sets physical parameters (wheel distance and radius), limits for
 * acceleration/velocity and simple PID/follow defaults used by other
 * methods. These values are tuned for the Micro-Rato platform.
 */
robot_t::robot_t()
{
  wheel_dist = 0.125;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  follow_k = -0.15;
  follow_v = 0.20;
}


/**
 * @brief Estimate robot odometry from encoder deltas.
 *
 * This method converts encoder tick counts into wheel angular speeds,
 * linear wheel speeds, and then integrates the robot pose (x, y, theta)
 * using a simple differential-drive kinematic model. Currently not used
 * by the main loop but kept for future heading/pose estimation.
 */
void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}




/**
 * @brief Apply simple acceleration limits to requested v/w.
 *
 * Constrains the change in linear and angular velocity to maximum
 * increments (dv_max/dw_max) to avoid abrupt changes. This method
 * updates the current v and w based on v_req and w_req.
 */
void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


/**
 * @brief Convert robot linear/angular velocity (v,w) to motor references.
 *
 * Computes wheel linear velocity references and corresponding angular
 * speeds, then selects the appropriate control mode to fill PWM outputs.
 * The PID branch is commented out but left for reference.
 */
void robot_t::VWToMotorsVoltage(void)
{
  v1ref = v + w * wheel_dist / 2;
  v2ref = v - w * wheel_dist / 2; 
  
  w1ref = v1ref * wheel_radius;
  w2ref = v2ref * wheel_radius;

  if (control_mode == cm_pwm) {

    PWM_1 = PWM_1_req;  
    PWM_2 = PWM_2_req;  
  }

  //  else if (control_mode == cm_pid) {
  //   u1 = 0;
  //   u2 = 0;      

  //   if (v1ref != 0) u1 = PID[0].calc(v1ref, v1e);
  //   else PID[0].Se = 0;

  //   if (v2ref != 0) u2 = PID[1].calc(v2ref, v2e);
  //   else PID[1].Se = 0;

  //   PWM_1 = u1 / battery_voltage * 255;
  //   PWM_2 = u2 / battery_voltage * 255;
 // }
}


/**
 * @brief Write PWM values to motor driver pins with direction control.
 *
 * The motor driver is controlled by two pins per motor. This helper
 * clamps the PWM to a maximum allowed value and then writes the
 * appropriate analog values to produce forward, reverse or stop
 * behaviour depending on the sign of new_PWM.
 *
 * @param new_PWM Signed PWM value (negative for reverse, positive for forward)
 * @param pin_a First output pin for the motor (PWM)
 * @param pin_b Second output pin for the motor (PWM)
 */
void robot_t::setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = 200;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  
  if (new_PWM == 0) 
  {  // Both outputs 0 -> A = H, B = H
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255);

  } else if (new_PWM > 0) 
  {
    analogWrite(pin_a, 255 - new_PWM);
    analogWrite(pin_b, 255);

  } 
  else 
  {
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255 + new_PWM);
  }
}





/**
 * @brief Sum all IR sensor readings from the line sensor array.
 *
 * Useful as a simple check to detect whether the robot is currently
 * over a bright/white surface (large sum) or dark/line (small sum).
 *
 * @return int Sum of the 5 IR sensor values.
 */
int robot_t::IR_sum()
{
  return IRLine.IR_values[0] + IRLine.IR_values[1] + IRLine.IR_values[2] + IRLine.IR_values[3] + IRLine.IR_values[4];
}


 /**
  * @brief Simple PID-based line follower.
  *
  * Reads the 5-element IR line sensor array and computes a weighted error
  * that represents lateral displacement from the line center. A PID
  * controller (IRkp/IRki/IRkd) is applied to compute a correction that
  * adjusts left/right motor PWM values to keep the robot centered.
  *
  * If all sensors read above the threshold it stops the robot (end of line).
  */
 void robot_t::followLine() 
 {
    int IR_tresh = robot.IRLine.IR_tresh;
    
    if (robot.IR_sum() < 2500) {
        
        error = -1.1 * IRLine.IR_values[0] - 1.0 * IRLine.IR_values[1] + 1.0 * IRLine.IR_values[3] + 1.1 * IRLine.IR_values[4];
        Serial.printf(" %d ", error);

        
        integral += error;
        derivative = error - prevError;
        prevError = error;

        
        int correction  = (IRkp * error + IRki * integral + IRkd * derivative); //PID constants are defined in robot.h

       
        if (correction > 100) correction = 100; // Max correction limit
        else if (correction < -100) correction = -100; // Min correction limit

        // Apply the PID correction to the motor PWM values
        int right_PWM = follow_speed - correction; // Subtract from left motor
        int left_PWM = follow_speed + correction; // Add to right motor

        //Serial.printf(" %d ", correction);
        // Apply the PWM values to the motors
        PWM_1 = left_PWM;
        PWM_2 = right_PWM;
     
    }
    else 
    {
      integral = 0;
      PWM_1 = follow_speed;
      PWM_2 = follow_speed;
    }
    
    if(IRLine.IR_values[0] < IR_tresh && IRLine.IR_values[1] < IR_tresh && IRLine.IR_values[2] < IR_tresh && IRLine.IR_values[3] < IR_tresh && IRLine.IR_values[4] < IR_tresh)
    {
      stop(); 
    }
}

// bool robot_t::align() 
//  {
//       int IR_tresh = robot.IRLine.IR_tresh;
  
//       error = -1.1 * IRLine.IR_values[0] - 1.0 * IRLine.IR_values[1] + 1.0 * IRLine.IR_values[3] + 1.1 * IRLine.IR_values[4];

//       if(error < 400 )
//       {
//         return true;
//       }



      
//       integral += error;
//       derivative = error - prevError;
//       prevError = error;

      
//       int correction  = (IRkp * error + IRki * integral + IRkd * derivative); //PID constants are defined in robot.h

      
//       if (correction > 100) correction = 100; // Max correction limit
//       else if (correction < -100) correction = -100; // Min correction limit

//       // Apply the PID correction to the motor PWM values
//       int right_PWM = correction; // Subtract from left motor
//       int left_PWM = - correction; // Add to right motor

//       //Serial.printf(" %d ", correction);
//       // Apply the PWM values to the motors
//       PWM_1 = left_PWM;
//       PWM_2 = right_PWM;
     
    
// }

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------

 // Node functions (simple motion primitives)

 /**
  * @brief Stop both motors immediately (set PWM to 0).
  */
 void robot_t::stop()
 {
  robot.PWM_1 = 0;
  robot.PWM_2 = 0; 
 }

/**
 * @brief Execute a timed right turn primitive.
 *
 * Starts a turn where only one motor is driven. The routine uses an
 * internal static timer to measure turn duration; when the timeout is
 * reached it sets the global END_TURN flag and resets the timer.
 */
void robot_t::right_turn()
{
  static unsigned long start_time = 0;
    if (start_time == 0) {
        start_time = millis();
       PWM_1 = nominal_speed;
       PWM_2 = 0;
    }

    if (millis() - start_time > 800) { // 435ms to turn ~90º
        END_TURN = true;
        start_time = 0; // reset for next turn
    }
}



/**
 * @brief Execute a timed left turn primitive (mirror of right_turn).
 */
void robot_t::left_turn()
{
  static unsigned long start_time = 0;
    if (start_time == 0) {
        start_time = millis();
       PWM_1 = 0;
       PWM_2 = nominal_speed;
    }

    if (millis() - start_time > 800) { // 435ms to turn ~90º
        END_TURN = true;
        start_time = 0; // reset for next turn
    }
}

  /**
   * @brief Execute a timed U-turn primitive.
   *
   * Drives the motors in opposite directions to perform a fast in-place
   * rotation. Uses an internal static timer similar to `right_turn`.
   */
  void robot_t::u_turn()
  {
      static unsigned long start_time = 0;
    if (start_time == 0) {
        start_time = millis();
       PWM_1 = nominal_speed;
       PWM_2 = -nominal_speed;
    }

    if (millis() - start_time > 80) { // 833ms to turn ~180º
        END_TURN = true;
        start_time = 0; // reset for next turn
    }

  }
  /**
   * @brief Drive both motors in reverse at nominal speed.
   */
  void robot_t::reverse()
  {
    PWM_1 =  -nominal_speed;
    PWM_2 = -nominal_speed;
  }

  /**
   * @brief Drive both motors forward at nominal speed.
   */
  void robot_t::forward( )
  {
    PWM_1 =  nominal_speed;
    PWM_2 = nominal_speed;
  }
