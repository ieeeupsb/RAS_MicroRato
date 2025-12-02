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

#ifndef IRLINE_H
#define IRLINE_H

#include "Arduino.h"
#include <string.h>

/** @brief Number of IR sensors in the array (fixed at 5) */
#define IRSENSORS_COUNT 5

/**
 * @file IRLine.h
 * @brief Interface for reading and processing a line of IR sensors used by the
 * micro-rato robot for line following and node detection.
 *
 * This header declares the IRLine_t class which stores the raw sensor values
 * and provides helpers to read sensors, encode the sensor state, compute line
 * edges, calibrate sensors and detect nodes (crossings). 
 *
 * Sensor Layout:
 * +-------------------------------------------------+
 * |  [0]    [1]    [2]    [3]    [4]               |
 * |  Left          Center          Right           |
 * +-------------------------------------------------+
 * 
 * Each sensor outputs 0-1023 (10-bit ADC) where:
 * - Low values (< IR_tresh): White surface / no line
 * - High values (> IR_tresh): Black line detected
 * 
 * Usage summary:
 *  - call readIRSensors() periodically to sample the IR sensors into
 *    `IR_values`;
 *  - use calcIRLineEdgeLeft()/calcIRLineEdgeRight() to compute a line position
 *    estimate for PID control; and
 *  - call detectNode() to check for junctions on the line.
 *
 */
class IRLine_t
{
  public:
    /**
     * @brief Position estimates from edge detection algorithms
     *
     * These floating-point values represent the estimated line position:
     * - pos_left: Position calculated scanning left-to-right
     * - pos_right: Position calculated scanning right-to-left
     * - total: Sum of all adjusted sensor readings (signal strength)
     * 
     * Interpretation is project-specific but typically:
     * - Positive values: Line is to the right
     * - Negative values: Line is to the left
     * - Magnitude: Distance from center
     */
    float pos_left, pos_right, total;

    /**
     * @brief Raw ADC readings from the line IR sensors
     *
     * Array of 5 sensor values read through the analog multiplexer.
     * Index mapping: [0]=leftmost, [2]=center, [4]=rightmost
     * 
     * Value range: 0-1023 (10-bit ADC)
     * - Values are inverted in readIRSensors() so high = black line
     * - Compare against IR_tresh to determine black/white
     * 
     * @note Values are updated each time readIRSensors() is called
     */
    int IR_values[IRSENSORS_COUNT];

    /** 
     * @brief Baseline offset subtracted from raw readings
     * 
     * This "water level" represents the minimum signal floor.
     * Sensor readings have IR_WaterLevel subtracted before processing
     * to normalize the values.
     * 
     * @note Currently initialized to 0 (no offset)
     */
    int IR_WaterLevel;

    /**
     * @brief Threshold used to decide if a sensor sees a black surface
     *
     * Default initialized to 850 in the implementation. 
     * Values greater than this threshold indicate a black (line) reading.
     * 
     * Usage: `if (IR_values[i] > IR_tresh) { * black detected * }`
     * 
     * @note May need calibration for different lighting conditions
     * @note Original default was 700, currently set to 850
     */
    int IR_tresh = 850;
    
    /** 
     * @brief Maximum sensor reading from most recent scan
     * 
     * Updated by edge detection functions to track the strongest signal.
     * Can be used to assess line contrast or sensor health.
     */
    int IR_max;

    /** 
     * @brief Number of sensor threshold crossings detected (internal use)
     * 
     * Used internally by edge detection algorithms to count
     * transitions from white to black as sensors are scanned.
     */
    int crosses;

    /** 
     * @brief Crossing counters for junction detection
     * 
     * - cross_count: Current number of crossings in this scan
     * - last_cross_count: Previous scan's crossing count
     * 
     * Used to detect changes in line pattern between samples.
     */
    int cross_count, last_cross_count;

    /** 
     * @brief Threshold for deciding when a crossing/junction is present
     * 
     * When cross_count exceeds this value, a junction may be detected.
     * Default: 3
     */
    int cross_tresh;

    /** 
     * @brief ADC level considered a "black cross" for node detection
     * 
     * Calibration marker: 2.8 (project-specific units)
     * Used as a reference level for junction classification.
     */
    int black_cross_level;

    /** 
     * @brief Count of black sensors detected (junction detection)
     * 
     * Tracks how many sensors are currently reading black.
     * Used to distinguish between different junction types.
     */
    float blacks;

    /**
     * @brief Construct a new IRLine_t instance
     *
     * Initializes internal counters and calibration values:
     * - IR_WaterLevel = 0
     * - IR_tresh = 700 (threshold for black detection)
     * - cross_tresh = 3
     * - black_cross_level = 2.8
     * 
     * See implementation in IRLine.cpp for details.
     */
    IRLine_t();

    /**
     * @brief Read all IR sensors and store results in `IR_values`
     *
     * Samples all 5 sensors through the analog multiplexer:
     * 1. Sets MUX channel (3-7)
     * 2. Waits 100µs for signal stabilization
     * 3. Reads ADC value
     * 4. Inverts value (1023 - raw) so high = black
     * 5. Stores in IR_values array in reversed order
     * 
     * @note Call this once per main loop iteration
     * @note Updates IR_values array in-place
     */
    void readIRSensors(void);

    /**
     * @brief Compress 5 sensor readings into a single 32-bit value
     *
     * Each 10-bit reading is reduced to 6 bits and packed:
     * - Bits [29:24] = IR_values[0] >> 4
     * - Bits [23:18] = IR_values[1] >> 4
     * - Bits [17:12] = IR_values[2] >> 4
     * - Bits [11:6]  = IR_values[3] >> 4
     * - Bits [5:0]   = IR_values[4] >> 4
     * 
     * Useful for:
     * - Compact logging/transmission
     * - State comparison
     * - Pattern matching
     * 
     * @return uint32_t Packed sensor state (30 bits used)
     * @note Lossy compression: 4 LSBs are discarded per sensor
     */
    uint32_t encodeIRSensors(void);

    /**
     * @brief Print the current IR state over Serial for debugging
     *
     * Prints all 5 raw sensor values in a readable format.
     * Output format: "val0    val1    val2    val3    val4"
     * 
     * @note Requires Serial to be initialized with Serial.begin()
     * @note Useful for sensor calibration and troubleshooting
     */
    void printIRLine(void);
    
    /**
     * @brief Calibrate sensors by sampling background and line values
     *
     * Should sample sensors over white and black surfaces to determine:
     * - Optimal IR_tresh value
     * - IR_WaterLevel baseline
     * - IR_max expected range
     * 
     * Call this at startup or when lighting conditions change.
     * 
     * @note Currently a stub (not implemented)
     * @todo Implement calibration routine:
     *       1. Read sensors over white surface (take min)
     *       2. Read sensors over black line (take max)
     *       3. Set IR_tresh to midpoint
     */
    void calibrate(void);
    
    /**
     * @brief Compute line edge/position using left-to-right sensor scan
     *
     * Algorithm:
     * 1. Scan sensors left to right (0→4)
     * 2. Find first threshold crossing (white→black transition)
     * 3. Interpolate exact edge position between sensors
     * 4. Store result in pos_left
     * 
     * Used by PID controller for line following.
     * 
     * @note Updates pos_left, total, and IR_max
     * @note Returns position in arbitrary units (typically -32 to +32)
     */
    void calcIRLineEdgeLeft(void);

    /**
     * @brief Compute line edge/position using right-to-left sensor scan
     *
     * Same algorithm as calcIRLineEdgeLeft() but scanning from right to left.
     * Provides alternative position estimate for comparison or averaging.
     * 
     * @note Updates pos_right, total, and IR_max
     */
    void calcIRLineEdgeRight(void);
    
    /**
     * @brief Detect a junction/node and return a character code
     *
     * Analyzes the 5 sensor pattern to classify the current position:
     * 
     * Return Codes:
     * - 'L': Left junction (XXXOO) - left 3 black, right 2 white
     * - 'R': Right junction (OOXXX) - left 2 white, right 3 black
     * - 'B': Both sides black (XXXXX) - T-junction, cross, or end
     * - 'W': White/off line (OOOOO) - after turn or dead end
     * - 'N': Normal line (OOXOO, OXOOO, etc.) - single sensor patterns
     * - 'E': Error/unstable - pattern not stable for 40 cycles
     * 
     * Stability Requirement:
     * - Same pattern must be detected for 40 consecutive calls
     * - This prevents false positives from noise or transitions
     * - Returns 'E' until stability achieved
     * 
     * @return char Junction type code or 'E' if unstable
     * @note 40-cycle stability = ~40ms at typical loop rates
     * @note Critical for reliable junction detection
     */
    char detectNode();

};

#endif // IRLINE_H