/**
 * @file state_machines.h
 * @brief State machine type definitions and handler declarations
 * 
 * This file defines the states for all finite state machines used by the robot:
 * - Main FSM: Overall operation mode (idle, mapping, solving)
 * - Map FSM: Maze exploration behavior
 * - Solve FSM: Solution execution behavior
 * - Test FSM: Development and debugging
 */

#ifndef STATE_MACHINES_H
#define STATE_MACHINES_H

// ============================================================================
// TIMER STRUCTURE
// ============================================================================

/**
 * @brief Simple timer block for state machine timing
 * 
 * @note Currently declared but not used in the code.
 *       Intended for time-based state transitions.
 */
typedef struct {
  unsigned char on;      ///< Timer active flag (1 = running, 0 = stopped)
  unsigned int time;     ///< Accumulated time in milliseconds
} timerBlock;


// ============================================================================
// MAIN STATE MACHINE
// ============================================================================

/**
 * @brief Top-level operating modes
 * 
 * Controls the overall robot behavior:
 * - IDLE_MAIN: Waiting for user to start
 * - MAP: Exploring and learning the maze
 * - READY: Finished mapping, waiting to solve
 * - SOLVE: Executing the optimized solution
 * - SOLVED: Finished solution, waiting for reset/restart
 */
typedef enum {
  IDLE_MAIN,       ///< Initial state, waiting for START button
  MAP,             ///< Mapping phase - explore maze
  READY,           ///< Mapping complete, ready to solve
  SOLVE,           ///< Solving phase - execute optimal path
  SOLVED           ///< Solution complete
} StateNamesMain;


// ============================================================================
// MAPPING STATE MACHINE
// ============================================================================

/**
 * @brief States for maze exploration
 * 
 * Implements a wall-following algorithm to map the maze:
 * - Follows the line using PID control
 * - Detects junctions and turns accordingly
 * - Records path taken (TODO: not fully implemented)
 */
typedef enum {
  IDLE_MAP,         ///< Inactive, waiting for MAP mode
  FOLLOW_LINE_MAP,  ///< Following line with PID control
  U_TURN,           ///< Executing 180° turn (dead end)
  LEFT_TURN_MAP,    ///< Executing 90° left turn
  RIGHT_TURN_MAP,   ///< Executing 90° right turn
  REVERSE,          ///< Reversing (currently unused)
  SMALL_FORWARD,    ///< Brief forward movement to clear junction
  FORWARD_MAP,      ///< Forward until next node (currently unused)
  END               ///< Mapping complete
} StateNamesMap;


// ============================================================================
// SOLVING STATE MACHINE
// ============================================================================

/**
 * @brief States for executing the optimized solution
 * 
 * Follows the instructions from the optimized path stack:
 * - Pops instructions one by one
 * - Executes turns and forward movements
 * - Completes when instruction stack is empty
 * 
 * @note Currently not fully integrated into main loop
 */
typedef enum {
  IDLE_SOLVE,         ///< Inactive, waiting for SOLVE mode
  FOLLOW_LINE_SOLVE,  ///< Following line between instructions
  GET_INSTRUCTION,    ///< Fetching next move from solution
  LEFT_TURN_SOLVE,    ///< Executing solution left turn
  RIGHT_TURN_SOLVE,   ///< Executing solution right turn
  FORWARD_SOLVE,      ///< Moving forward through junction
  FINISH              ///< Solution execution complete
} StateNamesSolve;


// ============================================================================
// TEST STATE MACHINE (Development/Debugging)
// ============================================================================

/**
 * @brief States for testing and development
 * 
 * Similar to Map FSM but with additional debugging features
 * and the ability to pause/resume with the START button.
 * 
 * @note Currently commented out in main loop
 */
typedef enum {
  FOLLOW_TEST,        ///< Following line in test mode
  FORWARD_TEST,       ///< Moving forward (unused)
  SMALL_FORWARD_TEST, ///< Brief forward movement
  RIGHT_TURN_TEST,    ///< Testing right turns
  LEFT_TURN_TEST,     ///< Testing left turns
  BACKWARD_TEST,      ///< Reversing to previous junction
  U_TURN_TEST,        ///< Testing U-turns
  STOP_TEST,          ///< Stopped, waiting for resume
  END_TEST,           ///< Test complete
  ALIGNING_TEST,      ///< Alignment test (unused)
  IDLE_TEST           ///< Test mode inactive
} StateNamesTest;


// ============================================================================
// SIMPLE FOLLOW STATE MACHINE
// ============================================================================

/**
 * @brief Minimal state machine for basic line following
 * 
 * @note Named "FODASE" (Portuguese slang, roughly "whatever")
 *       suggesting this was a quick test implementation.
 *       Consider renaming to StateNamesBasicFollow for clarity.
 */
typedef enum {
  FOLLOW_FODASE  ///< Just follow the line, ignore everything else
} StateNamesFodase;


// ============================================================================
// GLOBAL FLAGS
// ============================================================================

/** @brief Flag indicating mapping phase is complete */
extern bool END_MAP;

/** @brief Flag indicating solving phase is complete */
extern bool END_SOLVE;


// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Timer utility functions (currently stubs)
void update_timers();            // Timer update (not implemented)
void start_timer(timerBlock* t); // Start a timer (not implemented)
void stop_timer(timerBlock* t);  // Stop a timer (not implemented)

// State machine handlers
void main_FSM_handler();   // Top-level mode control
void map_FSM_handler();    // Maze exploration
void solve_FSM_handler();  // Solution execution
void test_FSM_handler();   // Testing/debugging

#endif // STATE_MACHINES_H