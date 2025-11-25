
#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <math.h>
//#include "PID.h"
#include "IRLine.h"
#include "state_machines.h"

/**
 * @file robot.h
 * @brief Public interface and data structures for the robot low-level control.
 *
 * This header declares the `robot_t` class used across the project to hold
 * encoder state, simple motion primitives, PID parameters for line following
 * and helpers to convert requested linear/angular velocities into motor
 * commands. The implementation lives in `robot.cpp`.
 */

#ifndef NUM_WHEELS
#define NUM_WHEELS 2
#endif

/**
 * @brief Motor control mode selector.
 *
 * - cm_pwm: direct PWM control (open-loop)
 * - cm_pid: closed-loop PID velocity control (unused/commented in code)
 */
typedef enum {
  cm_pwm,
  cm_pid
} control_mode_t;

/**
 * @brief Main robot class containing sensors, state and motion helpers.
 *
 * The class is intentionally simple: public member variables are used to
 * make the state accessible from the state machines and the `main` loop.
 */
class robot_t {
public:

  /* Encoder and odometry state ------------------------------------------------*/
  int enc1, enc2;            ///< Raw encoder deltas (ticks)
  float w1e, w2e;           ///< Estimated wheel angular velocities (rad/s)
  float v1e, v2e;           ///< Estimated wheel linear velocities (m/s)
  float ve, we;             ///< Estimated robot linear and angular velocity
  float ds, dtheta;         ///< Last integrated distance and angle deltas
  float rel_s, rel_theta;   ///< Relative displacement since last reset
  float xe, ye, thetae;     ///< Estimated pose (x, y, theta)

  /* Turning flags and timers --------------------------------------------------*/
  bool END_TURN = false;             ///< Set when a timed turn finishes
  unsigned long turn_start_time = 0; ///< Internal start timestamp for primitives
  bool is_turning = false;           ///< Helper flag to indicate turning

  /* Control and kinematic parameters -----------------------------------------*/
  float dt;                  ///< Control loop timestep (s)
  float v, w;                ///< Current linear (m/s) and angular (rad/s) speed
  float v_req, w_req;        ///< Requested linear and angular speeds
  float dv_max, dw_max;      ///< Max per-step change for acceleration limiting

  float wheel_radius, wheel_dist; ///< Physical parameters (m)

  /* Motor references and control ---------------------------------------------*/
  float v1ref, v2ref;        ///< Desired wheel linear speeds (m/s)
  float w1ref, w2ref;        ///< Desired wheel angular speeds (rad/s)
  float u1, u2;              ///< Low-level control outputs (e.g., voltage)
  int PWM_1, PWM_2;          ///< Current PWM outputs for the two motors
  int PWM_1_req, PWM_2_req;  ///< Requested PWM values (open-loop)
  control_mode_t control_mode;///< Active control mode (cm_pwm or cm_pid)

  /* Line follower PID parameters --------------------------------------------*/
  double IRkp = 0.12, IRki = 0, IRkd = 0.32; ///< PID gains used by followLine()
  double lastIRkp = IRkp, lastIRki = IRki, lastIRkd = IRkd; ///< Last-applied gains

  /* PID internal state -------------------------------------------------------*/
  double error;         ///< Current lateral error reported by IR sensors
  double prevError = 0; ///< Previous error for derivative term
  double integral = 0;  ///< Integrator state for I term
  double derivative = 0;///< Derivative term value

  float right_v = 0.0, left_v = 0.0, right_w = 0.0, left_w = 0.0; ///< wheel speeds

  float follow_v, follow_k; ///< Follow-line tuning parameters (unused in header)

  //PID_t PID[NUM_WHEELS];  // Optional PID controllers (commented)
  float battery_voltage;   ///< Measured battery voltage (used for closed-loop)
  int button_state;        ///< Simple input/button state
  IRLine_t IRLine;         ///< IR line-sensor array state

  /**
   * @brief Construct and initialize default robot parameters.
   *
   * See `robot.cpp` for the concrete default values used.
   */
  robot_t();

  /**
   * @brief Update odometry estimates from encoder deltas.
   */
  void odometry(void);

  /**
   * @brief Set requested linear/angular velocity (convenience wrapper).
   *
   * @param Vnom Requested linear velocity (m/s)
   * @param Wnom Requested angular velocity (rad/s)
   */
  void setRobotVW(float Vnom, float Wnom);

  /**
   * @brief Constrain acceleration to dv_max/dw_max.
   */
  void accelerationLimit(void);

  /**
   * @brief Convert V/W to wheel references and update PWM outputs.
   */
  void VWToMotorsVoltage(void);

  /**
   * @brief Helper to write clamped PWM values to a motor driver pair.
   *
   * See `robot.cpp::setMotorPWM` for implementation details.
   */
  void setMotorPWM(int new_PWM, int pin_a, int pin_b);

  /**
   * @brief Follow the line using the IR sensor array (right/left variants
   * are declared for completeness; implementation may be in .cpp).
   */
  void followLineRight(float Vnom, float K);
  void followLineLeft(float Vnom, float K);
  void followLine();
  //bool align();

  /* Simple motion primitives -------------------------------------------------*/
  void stop();
  void left_turn(); 
  void right_turn();
  void u_turn();
  void reverse();
  void forward();

  /**
   * @brief Sum of the IR sensor values (useful to detect line/no-line).
   * @return int Sum of the five IR sensors.
   */
  int IR_sum();
};

extern robot_t robot;


#endif // ROBOT_H
