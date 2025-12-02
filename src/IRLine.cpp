/* Copyright (c) 2019  Paulo Costa
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

  
#include "IRLine.h"
#include "Arduino.h"
#include <string.h>
#include "hardware_config.h"
#include "robot.h"

#define DEBUG 
#define Raspberry 
// #define Arduino 

/**
 * @file IRLine.cpp
 * @brief Implements infrared line sensor handling and line detection logic for the robot.
 *
 * This module manages all operations related to the infrared (IR) line sensors, including:
 *  - Reading sensor values through an analog multiplexer
 *  - Calculating left and right line edges
 *  - Encoding sensor data into a compact format
 *  - Detecting line patterns and junction nodes (left, right, T, cross, etc.)
 *
 * It supports both Arduino and Raspberry Pi Pico platforms through conditional compilation.
 * The functions use threshold-based analysis to interpret the reflectance readings and identify
 * navigation cues for line-following or maze-solving behaviors.
 *
 * @note Requires `robot.h`, `hardware_config.h`, and the appropriate hardware configuration for the MUX pins.
 *
*/


extern robot_t robot;

/**
 * @brief Construct a new IRLine_t object and initialize defaults.
 *
 * Initializes calibration and detection thresholds and clears working
 * variables. These defaults are conservative starting values and may be
 * adjusted later by a call to `calibrate()`.
 *
 * Defaults set in the implementation:
 *  - `IR_WaterLevel = 0`: No baseline offset
 *  - `IR_tresh = 700`: Threshold to classify black/white (0-1023 scale)
 *  - `cross_tresh = 3`: Threshold for crossing detection
 *  - `black_cross_level = 2.8`: Calibration level marker (project-specific)
 * 
 * @note These values work for typical indoor lighting but may need
 *       adjustment for different environments
 */
IRLine_t::IRLine_t()
{
  IR_WaterLevel = 0;        // Baseline offset (water level) subtracted from raw readings
  IR_tresh = 700;           // Threshold used to decide black vs white on a 10-bit ADC (0..1023)
  cross_tresh = 3;          // Crossing detection threshold (count of sensors/signals)
  black_cross_level = 2.8;  // Calibration marker for a black surface (project-specific units)
}

/**
 * @brief Calibrate the IR sensors.
 *
 * Intended to sample sensors under known black/white surfaces and adjust
 * internal thresholds (for example `IR_tresh` and `IR_max`).
 *
 * @note Currently this function is a stub (not implemented).
 * @todo Implement calibration routine:
 *       1. Prompt user to place robot on white surface
 *       2. Sample all sensors, record minimum values
 *       3. Prompt user to place robot on black line
 *       4. Sample all sensors, record maximum values
 *       5. Set IR_tresh to (min + max) / 2
 *       6. Store calibration data in EEPROM/Flash for persistence
 */
void IRLine_t::calibrate(void)
{
  // TODO: Implement calibration
}

/**
 * @brief Compute the left line edge using the left-to-right sensor scan.
 *
 * The algorithm scans the five IR sensors from left to right (index 0→4),
 * subtracts `IR_WaterLevel` from each reading, clamps negatives to zero,
 * and searches for the first threshold crossing where the previous sample
 * is below `IR_tresh` and the current sample is above `IR_tresh`.
 * 
 * When a crossing is found, an interpolated position is computed using
 * linear interpolation between the two sensor readings:
 * 
 * Position = base_position + offset * (threshold - last_value) / (value - last_value)
 * 
 * This gives sub-sensor resolution for smoother line following.
 *
 * Side effects:
 *  - Updates `pos_left`: Interpolated edge position
 *  - Updates `total`: Sum of adjusted readings (signal strength)
 *  - Updates `IR_max`: Maximum sensor reading found
 * 
 * @note Position units are arbitrary but consistent (-32 to +32 typical range)
 * @note If no edge found, pos_left remains at rightmost position (2 * 16.0)
 */
void IRLine_t::calcIRLineEdgeLeft(void)
{
  byte c;         // Sensor index (0-4)
  byte found;     // Flag indicating edge was detected
  int v;          // Current adjusted IR sensor reading
  int last_v;     // Previous adjusted IR sensor reading

  found = 0;
  IR_max = 0;
  pos_left = 2 * 16.0;  // Default to rightmost position if no edge found
  total = 0;            // Accumulate IR signal strength
  last_v = 0;
  
  // Scan sensors left to right
  for (c = 0; c < 5; c++) {
    // Adjust reading by subtracting baseline
    v = IR_values[c] - IR_WaterLevel;
    if (v < 0) v = 0;  // Clamp negative values
    
    // Track maximum reading
    if (v > IR_max) IR_max = v;
    
    // Accumulate total signal
    total = total + v;

    // Detect first white-to-black transition
    if (!found && last_v < IR_tresh && v > IR_tresh) {
      // Interpolate exact position between sensors
      // -12: offset for leftmost sensor
      // 16.0 * (c - 2): sensor position relative to center
      // Last term: interpolation factor based on threshold crossing
      pos_left = -12 + 16.0 * (c - 2) + 16.0 * (IR_tresh - last_v) / (v - last_v);
      found = 1; // Found the edge, stop searching
    }
    last_v = v;
  }
}

/**
 * @brief Compute the right line edge by scanning sensors right-to-left.
 *
 * Same algorithm as `calcIRLineEdgeLeft()` but scanning from the outer
 * right sensor towards the left (index 4→0). This provides an alternative
 * edge position estimate that can be:
 * - Used alone for right-edge following
 * - Averaged with pos_left for center following
 * - Compared with pos_left to detect line width
 * 
 * Results are stored in `pos_right` with opposite sign convention.
 *
 * Side effects:
 *  - Updates `pos_right`: Interpolated edge position
 *  - Updates `total`: Sum of adjusted readings
 *  - Updates `IR_max`: Maximum sensor reading
 * 
 * @note Position is negated at the end for consistent coordinate system
 */
void IRLine_t::calcIRLineEdgeRight(void)
{
  byte c;         // Sensor index (reverse scan: 0-4 maps to 4-0)
  byte found;     // Flag indicating edge was detected
  int v;          // Current adjusted IR sensor reading
  int last_v;     // Previous adjusted IR sensor reading

  found = 0;
  IR_max = 0;
  pos_right = -2 * 16.0;  // Default to leftmost position if no edge found
  total = 0;
  last_v = 0;
  
  // Scan sensors right to left (reverse order)
  for (c = 0; c < 5; c++) {
    // Access sensors in reverse: [4], [3], [2], [1], [0]
    v = IR_values[4 - c] - IR_WaterLevel;
    if (v < 0) v = 0;
    
    if (v > IR_max) IR_max = v;
    total = total + v;

    // Detect first white-to-black transition (from right)
    if (!found && last_v < IR_tresh && v > IR_tresh) {
      // Same interpolation logic, but negated for right side
      pos_right = -(-12 + 16.0 * (c - 2) + 16.0 * (IR_tresh - last_v) / (v - last_v));
      found = 1;
    }
    last_v = v;
  }
}


/**
 * @brief Set the external analog multiplexer input channel.
 *
 * Writes the appropriate binary values to the three MUX control pins (A,B,C)
 * so the external multiplexer connects the desired channel to the ADC pin.
 * Platform-specific GPIO calls are used under conditional compilation.
 * 
 * Channel selection truth table:
 * +-----+-----+-----+----------+
 * |  C  |  B  |  A  | Channel  |
 * +-----+-----+-----+----------+
 * |  0  |  0  |  0  |    0     |
 * |  0  |  0  |  1  |    1     |
 * |  0  |  1  |  0  |    2     |
 * |  0  |  1  |  1  |    3     |
 * |  1  |  0  |  0  |    4     |
 * |  1  |  0  |  1  |    5     |
 * |  1  |  1  |  0  |    6     |
 * |  1  |  1  |  1  |    7     |
 * +-----+-----+-----+----------+
 * 
 * IR sensors are connected to channels 3-7 of the multiplexer.
 *
 * @param channel MUX channel number (typically 0..7)
 * @note Uses platform-specific GPIO functions (Raspberry Pi Pico or Arduino)
 */
static void adc_set_channel(int channel) 
{
  #ifdef Raspberry
  // Raspberry Pi Pico: Set multiple GPIO pins simultaneously using bitmask
  // This function (from the Raspberry Pi Pico SDK) sets several GPIO pins at once
  gpio_put_masked(digitalPinToBitMask(MUXA_PIN) | 
                  digitalPinToBitMask(MUXB_PIN) | 
                  digitalPinToBitMask(MUXC_PIN), 
                  channel << MUXA_PIN);
  #endif
  
  #ifdef Arduino
  // Arduino: Set each pin individually
  digitalWrite(MUXA_PIN, channel & 1);         // Bit 0
  digitalWrite(MUXB_PIN, (channel >> 1) & 1);  // Bit 1
  digitalWrite(MUXC_PIN, (channel >> 2) & 1);  // Bit 2
  #endif
}

/**
 * @brief Read a single ADC sample from the specified MUX channel.
 *
 * The function sets the MUX channel, waits a short time for the signal to
 * settle, and then reads the value from analog input A2 where the MUX is
 * connected.
 * 
 * Timing:
 * - MUX switching time: ~1µs
 * - Settling time: 100µs (conservative to ensure stable reading)
 * - ADC conversion time: ~100µs (hardware dependent)
 * 
 * Total time per sensor: ~200µs
 * Total time for 5 sensors: ~1ms
 *
 * @param channel MUX channel number to read (0-7)
 * @return uint16_t ADC reading, range 0..1023 (10-bit ADC)
 * @note A delayMicroseconds(100) is used to allow the multiplexer output to
 *       settle before sampling. May be reduced if faster scanning needed.
 */
uint16_t read_adc(int channel)
{
  adc_set_channel(channel);  // Switch external MUX to the desired channel
  delayMicroseconds(100);    // Wait for signal to settle (may be adjustable)
  return analogRead(A2);     // The MUX output connects to analog input A2
}

/**
 * @brief Read all IR sensors via the analog multiplexer and store values.
 *
 * The code reads MUX channels 3..7 (where the 5 IR sensors are connected),
 * inverts the ADC value (1023 - raw) and stores the results into the
 * robot's IRLine array in reversed order.
 * 
 * Why invert?
 * - Raw ADC: high voltage = reflected light = white surface
 * - Inverted: high value = dark surface = black line
 * - This matches the intuitive convention for line following
 * 
 * Why reverse order?
 * - MUX channels 3-7 map to sensors in one physical order
 * - Array indices 0-4 represent left-to-right
 * - Reversal aligns physical and logical ordering
 * 
 * Sensor mapping:
 * - MUX Ch 7 → IR_values[0] (leftmost)
 * - MUX Ch 6 → IR_values[1]
 * - MUX Ch 5 → IR_values[2] (center)
 * - MUX Ch 4 → IR_values[3]
 * - MUX Ch 3 → IR_values[4] (rightmost)
 *
 * Side effects: writes into `robot.IRLine.IR_values[]`.
 * 
 * @note Total execution time: ~1ms for all 5 sensors
 * @note Call once per main loop iteration
 */
void IRLine_t::readIRSensors(void)
{
  byte c;  // Read the five IR sensors using the AD converter
  for (c = 0; c < IRSENSORS_COUNT; c++) 
  {
    // Read MUX channels 3-7, invert values, store in reverse order
    robot.IRLine.IR_values[(IRSENSORS_COUNT - 1) - c] = 1023 - read_adc(3 + c);
  }
}

/**
 * @brief Encode the five IR sensor readings into a compact 32-bit value.
 *
 * Each sensor reading is a 10-bit value (0..1023). This function reduces
 * each reading to 6 bits by right-shifting 4 bits (lossy compression) and
 * packs the five 6-bit values into a 32-bit integer (30 bits used total).
 * 
 * Why compress?
 * - Compact representation for logging/transmission
 * - Fast comparison for pattern matching
 * - Efficient storage for path recording
 * 
 * Bit layout (from most to least significant bits):
 *  - bits [29:24] = IR_values[0] >> 4 (leftmost sensor)
 *  - bits [23:18] = IR_values[1] >> 4
 *  - bits [17:12] = IR_values[2] >> 4 (center sensor)
 *  - bits [11:6]  = IR_values[3] >> 4
 *  - bits [5:0]   = IR_values[4] >> 4 (rightmost sensor)
 * 
 * Example:
 * IR_values = [1000, 800, 200, 900, 1023]
 * After >> 4 = [62, 50, 12, 56, 63]
 * Encoded = 0b111110_110010_001100_111000_111111
 * 
 * Trade-off:
 * - Resolution reduced from 10 bits to 6 bits per sensor
 * - Still sufficient for node detection (64 levels vs 1024)
 * - 83% size reduction (160 bits → 30 bits)
 *
 * @return uint32_t Packed sensor state (range: 0 to 2^30-1)
 *                  useful for logging or compact comparisons
 */
uint32_t IRLine_t::encodeIRSensors(void)
{
  byte c;
  // Start with leftmost sensor, reduced to 6 bits
  uint32_t result = robot.IRLine.IR_values[0] >> 4;  // From 10 bits to 6 bits
  
  // Pack remaining sensors: shift left 6 bits, OR with next sensor
  for (c = 1; c < 5; c++)
  {
    result = (result << 6) | (robot.IRLine.IR_values[c] >> 4);
  }
  return result;
}


/**
 * @brief Print the current IR sensor values to Serial for debugging.
 * 
 * Outputs all 5 sensor values in a readable format:
 * "val0     val1     val2     val3     val4"
 * 
 * Useful for:
 * - Checking sensor connectivity
 * - Calibrating IR_tresh
 * - Debugging junction detection
 * - Verifying sensor alignment
 *
 * @note Serial must be initialized by the caller (Serial.begin) before
 *       calling this function.
 * @note Values are printed with fixed spacing for alignment
 */
void IRLine_t::printIRLine(void)
{
  Serial.print(robot.IRLine.IR_values[0]);
  Serial.print("               ");
  Serial.print(robot.IRLine.IR_values[1]);
  Serial.print("               ");
  Serial.print(robot.IRLine.IR_values[2]);
  Serial.print("               ");
  Serial.print(robot.IRLine.IR_values[3]);
  Serial.print("               ");
  Serial.println(robot.IRLine.IR_values[4]);
}


/**
 * @brief Analyze the five IR sensors and classify the current "node" (road pattern).
 * 
 * This is the core junction detection algorithm. It examines all 5 sensors
 * and determines what type of line pattern is present.
 * 
 * Detection Patterns and Return Codes:
 * +----------+----------+-------------------------------+---------+
 * | Code     | Pattern  | Meaning                       | Action  |
 * +----------+----------+-------------------------------+---------+
 * | 'L'      | XXXOO    | Left junction                 | Turn L  |
 * | 'R'      | OOXXX    | Right junction                | Turn R  |
 * | 'B'      | XXXXX    | Both/All black                | Decide  |
 * |          |          | (T-junction/Cross/End)        |         |
 * | 'W'      | OOOOO    | White/No line                 | U-turn  |
 * | 'N'      | OOXOO    | Normal line (single sensor)   | Follow  |
 * |          | OXOOO    | (various single patterns)     |         |
 * |          | OOOXO    |                               |         |
 * |          | XOOOO    |                               |         |
 * |          | OOOOX    |                               |         |
 * | 'E'      | Other    | Error/Unstable                | Wait    |
 * +----------+----------+-------------------------------+---------+
 * 
 * Legend: X = black (> IR_tresh), O = white (< IR_tresh)
 * 
 * Stability Filtering:
 * - Same pattern must be detected for 40 consecutive calls
 * - This prevents false positives from:
 *   * Sensor noise
 *   * Transition between states
 *   * Shadows or reflections
 * - Returns 'E' until stability threshold reached
 * 
 * Why 40 cycles?
 * - At ~1ms loop time, 40 cycles ≈ 40ms
 * - Typical robot movement: 0.5 m/s → 20mm in 40ms
 * - Junction width: ~50mm
 * - This ensures robot is fully on the junction before detection
 * 
 * @return char Node code as described above. Returns 'E' until a stable node
 *              is observed for the required number of cycles.
 * 
 * @note Critical function for navigation - must be very reliable
 * @note Consider making required_stability configurable for tuning
 */
char IRLine_t::detectNode()
{
  // Static variables persist between calls for stability tracking
  static char last_node = 'E';           // Previously detected node type
  static int stability_count = 0;        // Consecutive identical readings
  static int required_stability = 40;    // Threshold for confirming detection
  
  char current_node;  // Node type detected in this call

  // ====================================================================
  // PATTERN MATCHING - Classify sensor pattern
  // ====================================================================
  
  // Left junction: XXXOO (left 3 black, right 2 white)
  if (IR_values[0] > IR_tresh && 
      IR_values[1] > IR_tresh && 
      IR_values[2] > IR_tresh && 
      IR_values[3] < IR_tresh && 
      IR_values[4] < IR_tresh)
  {
    current_node = 'L';
  }
  
  // Right junction: OOXXX (left 2 white, right 3 black)
  else if (IR_values[0] < IR_tresh && 
           IR_values[1] < IR_tresh && 
           IR_values[2] > IR_tresh && 
           IR_values[3] > IR_tresh && 
           IR_values[4] > IR_tresh)
  {
    current_node = 'R';
  }
  
  // All black: XXXXX (T-junction, Cross, or End)
  else if (IR_values[0] > IR_tresh && 
           IR_values[1] > IR_tresh && 
           IR_values[2] > IR_tresh && 
           IR_values[3] > IR_tresh && 
           IR_values[4] > IR_tresh)
  {
    current_node = 'B';
  }
  
  // All white: OOOOO (Off line, after turn, or dead end)
  else if (IR_values[0] < IR_tresh && 
           IR_values[1] < IR_tresh && 
           IR_values[2] < IR_tresh && 
           IR_values[3] < IR_tresh && 
           IR_values[4] < IR_tresh)
  {
    current_node = 'W';
  }
  
  // Normal line: Single sensor patterns
  // Matches any pattern with one sensor black, others white:
  // OXOOO, OOOXO, OOXOO, XOOOO, OOOOX
  else if ((IR_values[0] < IR_tresh && IR_values[1] > IR_tresh && IR_values[2] < IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh) || // OXOOO
           (IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] < IR_tresh && IR_values[3] > IR_tresh && IR_values[4] < IR_tresh) || // OOOXO
           (IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] > IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh) || // OOXOO
           (IR_values[0] > IR_tresh && IR_values[1] < IR_tresh && IR_values[2] < IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh) || // XOOOO
           (IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] < IR_tresh && IR_values[3] < IR_tresh && IR_values[4] > IR_tresh))   // OOOOX
  {
    current_node = 'N';
  }
  
  // Unrecognized pattern (e.g., XXOXX, OXOXO, etc.)
  else
  {
    current_node = 'E';
  }

  // ====================================================================
  // STABILITY FILTERING - Require consistent readings
  // ====================================================================
  
  // If the same node type is detected again → increase the stability counter.
  // If it changes → reset the counter and update last_node.
  if (current_node == last_node)
  {
    stability_count++;
  }
  else
  {
    stability_count = 0;        // Pattern changed, restart counting
    last_node = current_node;   // Update tracked pattern
  }

  // Only return a confirmed node when it has stayed stable for enough cycles
  if (stability_count >= required_stability)
  {
    #ifdef DEBUG
    Serial.print("Node: ");
    Serial.println(current_node);
    #endif
    return current_node;  // Confident detection
  }

  // Return 'E' (Error/Unknown) until a node is stable enough
  // This prevents the state machine from reacting to transient patterns
  return 'E';
}