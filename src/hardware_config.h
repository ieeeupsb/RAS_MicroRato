/**
 * @file hardware_config.h
 * @brief Hardware pin definitions and configuration constants
 * 
 * This file centralizes all hardware-specific definitions including:
 * - GPIO pin assignments
 * - Speed parameters
 * - Sensor configuration
 * 
 * Modify this file when changing physical connections or tuning performance.
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// ============================================================================
// USER INTERFACE PINS
// ============================================================================

/** @brief Start button - triggers mapping/solving phases */
#define START_BUTTON 7

/** @brief Reset button - returns to initial state */
#define RESET_BUTTON 27


// ============================================================================
// ENCODER PINS (Currently unused in code)
// ============================================================================

/** @brief Encoder 1 - Channel A (left wheel) */
#define ENC1_A 2

/** @brief Encoder 1 - Channel B (left wheel) */
#define ENC1_B 3

/** @brief Encoder 2 - Channel A (right wheel) */
#define ENC2_A 0

/** @brief Encoder 2 - Channel B (right wheel) */
#define ENC2_B 1


// ============================================================================
// MOTOR DRIVER PINS
// ============================================================================

/** @brief Motor 1 (left) - Driver pin A */
#define MOTOR1A_PIN 16

/** @brief Motor 1 (left) - Driver pin B */
#define MOTOR1B_PIN 17

/** @brief Motor 2 (right) - Driver pin A */
#define MOTOR2A_PIN 14

/** @brief Motor 2 (right) - Driver pin B */
#define MOTOR2B_PIN 15


// ============================================================================
// ADC MULTIPLEXER PINS
// ============================================================================

/** @brief ADC input pin (where multiplexer output connects) */
#define ADC_IN_PIN 28

/** @brief Multiplexer address bit A (LSB) */
#define MUXA_PIN 18

/** @brief Multiplexer address bit B */
#define MUXB_PIN 19

/** @brief Multiplexer address bit C (MSB) */
#define MUXC_PIN 20


// ============================================================================
// EXTERNAL CONTROL PINS
// ============================================================================

/** 
 * @brief Control pin for external device (purpose unclear)
 * @note This pin is defined but never used in the code
 */
#define TINY_CTRL_PIN 21


// ============================================================================
// SPEED CONFIGURATION
// ============================================================================

/**
 * @brief Speed for turns and forward movements (0-255 PWM)
 * 
 * Used during:
 * - Left/right turns
 * - U-turns
 * - Forward movements at junctions
 * 
 * @note Original comment suggests this should be FOLLOW_SPEED + 30
 *       Current value is 80 which equals FOLLOW_SPEED + 30
 */
#define NOMINAL_SPEED 80

/**
 * @brief Base speed for line following (0-255 PWM)
 * 
 * This is the baseline speed used during PID-controlled line following.
 * PID corrections are added/subtracted from this value.
 * 
 * Tested values:
 * - 50: Current setting (stable)
 * - 80: Mentioned in comments (may be too fast)
 */
#define FOLLOW_SPEED 50


#endif // HARDWARE_CONFIG_H