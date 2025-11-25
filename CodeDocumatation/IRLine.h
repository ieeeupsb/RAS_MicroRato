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

#define IRSENSORS_COUNT 5
/**
 * @brief
 * IR_values is a vector the will save all the value of each IRSensors
 */
class IRLine_t
{
  public:
    float pos_left, pos_right, total;
    int IR_values[IRSENSORS_COUNT];
    int IR_WaterLevel;
    int IR_tresh = 850, IR_max;

    int crosses;
    int cross_count, last_cross_count;
    int cross_tresh;
    int black_cross_level;

    float blacks;

    /**
   * @brief Inializando parameters
   * 
   */
    IRLine_t();


    /**
    * @brief This function reads all of the robot’s infrared line sensors and stores their values in an array.
    */
    void readIRSensors(void);

    /**
   * @brief  This function compresses them to 6 bits each and stores all five inside one 32-bit number
   * You have 5 IR sensors, each giving a 10-bit reading (from analogRead(), range 0–1023).
   */
    uint32_t encodeIRSensors(void);
 
    /**
   * @brief  This function compresses them to 6 bits each and stores all five inside one 32-bit number
   * You have 5 IR sensors, each giving a 10-bit reading (from analogRead(), range 0–1023).
   */
    void printIRLine(void);
    

    /**
     * @brief Is to calibrate the IR sensor
     * 
     * Not impelemetet
     */
    void calibrate(void);
    
    /**
     * @brief This function is to find the left edge
     */
    void calcIRLineEdgeLeft(void);

    /**
     * @brief This function is to find the rigth edge
     * 
     */
    void calcIRLineEdgeRight(void);
    //void calcIRLineCenter(void);
    



    /**
   * Node detection
   * @brief This function analyzes the five IR sensors and classifies what kind of "node" (road pattern) the robot is currently over:
   * last_node{
   * E -> Erro or unknown
   * L -> Left junction
   * R -> Rigth junction
   * B → Both sides black (cross or T-junction)
   * W → White (no line, maybe after a junction or U-turn)
   * N → Normal line
   * }
   * 
   * | Condition          | Pattern                        | Meaning                              | Returned char |
  | ------------------ | ------------------------------ | ------------------------------------ | ------------- |
  | `XXXOO`            | Left 3 dark, right 2 light     | Left node detected                   | `'L'`         |
  | `OOXXX`            | Right 3 dark, left 2 light     | Right node detected                  | `'R'`         |
  | `XXXXX`            | All dark                       | Cross / T-junction / End             | `'B'`         |
  | `OOOOO`            | All light                      | Off the line (after turn or no line) | `'W'`         |
  | Single sensor dark | Regular line following pattern | `'N'`                                |               |
  | Anything else      | No clear match                 | `'E'`                                |               |

  */
    char detectNode();

};

#endif // IRLINE_H
