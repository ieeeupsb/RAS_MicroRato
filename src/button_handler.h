/**
 * @file button_handler.h
 * @brief Button edge detection for user interface
 * 
 * Provides rising edge detection for the START button.
 * This allows the state machines to respond only when the button
 * is first pressed, not while it's held down.
 */

#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

/**
 * @brief Detect rising edge (press) of START button
 * 
 * Call this function once per main loop iteration.
 * It updates the global re_START_BUTTON flag which is true
 * for exactly one loop cycle when the button is pressed.
 * 
 * @note No debouncing is implemented - may need to add delay
 *       if false triggers occur
 */
void edge_detection();

/** 
 * @brief Flag indicating START button was just pressed
 * 
 * This flag is set to 1 for one loop cycle when a rising edge
 * (button press) is detected, then returns to 0.
 * 
 * Usage: Check this flag in state machine transitions
 */
extern int re_START_BUTTON;

#endif // BUTTON_HANDLER_H