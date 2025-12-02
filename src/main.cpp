/**
 * @file main.cpp
 * @brief Main entry point for the line follower robot
 * 
 * This file contains the Arduino setup() and loop() functions.
 * It initializes all hardware and repeatedly calls the state machine handlers.
 * 
 * @note The main loop has no explicit timing - it runs as fast as possible.
 *       Consider adding a fixed time step for more predictable behavior.
 */

#include <Arduino.h>
#include "hardware_config.h"
#include "robot.h"
#include "button_handler.h"
#include "state_machines.h"

//#define DEBUG

/**
 * @brief Arduino setup function - runs once at startup
 * 
 * Initializes:
 * - Serial communication
 * - Button pins with pull-up resistors
 * - Encoder pins (currently unused)
 * - Motor driver pins
 * - ADC multiplexer control pins
 * - ADC resolution (10-bit)
 */
void setup()
{
  // Initialize serial communication (baud rate set by Arduino core)
  Serial.begin();
  
  // Configure user interface buttons
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);

  // Configure encoder inputs (currently not used in code)
  // TODO: Initialize PIO encoders with init_PIO_dual_encoders()
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  // Configure motor driver outputs
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);
  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);

  // Configure ADC multiplexer control pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  
  // NOTE: init_PIO_dual_encoders() is declared but never called
  // Encoders are currently not being used for odometry
  //init_PIO_dual_encoders(ENC1_A, ENC2_A);

  // Set ADC resolution to 10 bits (0-1023 range)
  analogReadResolution(10);

  #ifdef DEBUG
  Serial.println("Setup complete - Robot initialized");
  #endif
}

/**
 * @brief Arduino main loop - runs continuously
 * 
 * Execution order:
 * 1. Read all IR sensors
 * 2. Detect button edge transitions
 * 3. Execute main state machine (determines operating mode)
 * 4. Execute map state machine (if in mapping mode)
 * 5. Apply calculated PWM values to motors
 * 
 * @note Solve_FSM_Handler() and Test_FSM_Handler() are defined but not called
 * @note Loop timing is not controlled - consider adding delay for consistency
 */
void loop() 
{
  // Read IR sensor array through multiplexer
  robot.IRLine.readIRSensors();
  
  // Debugging: Print raw sensor values
  // robot.IRLine.printIRLine();
  // robot.IRLine.detectNode();

  // Detect button press/release events
  edge_detection();

  // Execute state machines in hierarchical order
  main_FSM_handler();    // Top-level mode control
  map_FSM_handler();     // Maze exploration logic
  
  // TODO: These state machines are defined but not executed:
  // solve_FSM_handler();  // Solution execution
  // test_FSM_handler();   // Testing/debugging mode

  // Apply motor control signals
  robot.setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
  robot.setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

  #ifdef DEBUG
  // Debugging: Print motor PWM values
  // Serial.printf("PWM1: %d, PWM2: %d\n", robot.PWM_1, robot.PWM_2);
  #endif
}