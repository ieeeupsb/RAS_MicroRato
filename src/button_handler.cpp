/**
 * @file button_handler.cpp
 * @brief Implementation of button edge detection
 */

#include "button_handler.h"
#include "hardware_config.h"

//#define DEBUG

// Global variables for edge detection
static int p_START_BUTTON = 0;    // Previous button state
int re_START_BUTTON = 0;          // Rising edge detected flag

/**
 * @brief Detect rising edge (LOW to HIGH transition) of START button
 * 
 * Algorithm:
 * 1. Read current button state
 * 2. Compare with previous state
 * 3. If previous was LOW (0) and current is HIGH (1), set rising edge flag
 * 4. Store current state for next comparison
 * 
 * @note The button uses INPUT_PULLUP, so:
 *       - HIGH (1) = button pressed
 *       - LOW (0) = button released
 * 
 * @note Debouncing consideration: Original code had commented debounce logic.
 *       May need to add debounce delay if mechanical bouncing causes issues.
 *       Typical debounce time: 50ms
 */
void edge_detection()
{
  // Read current button state
  bool currentButtonState = digitalRead(START_BUTTON);

  // Commented out debounce code from original:
  // static unsigned long lastDebounceTime = 0;
  // const unsigned long debounceDelay = 50;
  // if ((millis() - lastDebounceTime) > debounceDelay) {
  //   lastDebounceTime = millis();
  // }
  
  // Detect rising edge: previous LOW, current HIGH
  if (!p_START_BUTTON && currentButtonState) {
    re_START_BUTTON = true;   // Button was just pressed
  } 
  else {
    re_START_BUTTON = false;  // No edge detected
  }
  
  // Store current state for next comparison
  p_START_BUTTON = currentButtonState;
  
  #ifdef DEBUG
  Serial.printf("-- Edges re_BUTTON=%d p_BUTTON=%d\n", 
                re_START_BUTTON, p_START_BUTTON);
  #endif
}