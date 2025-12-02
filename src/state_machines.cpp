/**
 * @file state_machines.cpp
 * @brief State machine implementations
 * 
 * Contains the logic for all state machines that control robot behavior.
 * Each handler function checks the current state and determines:
 * 1. Next state based on conditions
 * 2. Actions to perform in current state
 */

// Compilation flags - control which features are active
#define RUN            // Enable main state machines
//#define SOLVE        // Enable solve phase (currently incomplete)
//#define TEST         // Enable test state machine
#define DEBUG          // Enable debug print statements
//#define SUPERDEBUG   // Enable verbose state transition logging

#include <Arduino.h>
#include "state_machines.h"
#include "robot.h"
#include "path_handler.h"
#include "button_handler.h"

// External references
extern robot_t robot;
extern int re_START_BUTTON;

// Path stacks for recording and optimizing the route
// TODO: These need to be populated during mapping
stack<char> path_taken;    // Records moves during mapping (L, R, F, U)
stack<char> solved_path;   // Optimized solution path

// Current state variables for each FSM
StateNamesMain currentStateMain = MAP;  // Start in mapping mode
StateNamesMap currentStateMap = IDLE_MAP;
StateNamesSolve currentStateSolve = IDLE_SOLVE;
StateNamesTest currentStateTest = FOLLOW_TEST;
StateNamesFodase currentStatefodase = FOLLOW_FODASE;

// Global flags
bool END_MAP = false;    // Set when maze mapping is complete
bool END_SOLVE = false;  // Set when solution execution is complete

// Junction detection state variables
bool cross = false;              // Indicates a cross junction detected
int counter_align = 0;           // Alignment counter (unused)
static unsigned long forwardStartTime = 0;  // Timer for SMALL_FORWARD state


// ============================================================================
// TIMER FUNCTIONS (Currently Stubs)
// ============================================================================

/**
 * @brief Update all active timers
 * 
 * @note Currently not implemented. Should be called once per loop iteration.
 *       Implementation should:
 *       1. Calculate cycle time (current_time - last_time)
 *       2. For each timer where timer.on == 1, add cycle_time to timer.time
 * 
 * @todo Implement cycle time calculation and timer increments
 */
void update_timers()
{
  // TODO: Implement timer update logic
  // Example implementation:
  // static unsigned long last_time = 0;
  // unsigned long current_time = millis();
  // unsigned int cycle_time = current_time - last_time;
  // last_time = current_time;
  // 
  // if (some_timer.on == 1) {
  //   some_timer.time += cycle_time;
  // }
}

/**
 * @brief Start a timer
 * @param t Pointer to timer block to start
 */
void start_timer(timerBlock* t)
{
  t->on = 1;
  t->time = 0;
}

/**
 * @brief Stop a timer
 * @param t Pointer to timer block to stop
 */
void stop_timer(timerBlock* t)
{
  t->on = 0;
  t->time = 0;
}


// ============================================================================
// MAIN STATE MACHINE - Overall Mode Control
// ============================================================================

#ifdef RUN
/**
 * @brief Main state machine handler
 * 
 * Controls high-level robot operation modes:
 * - IDLE_MAIN: Wait for start command
 * - MAP: Enable mapping state machine
 * - READY: Display/prepare optimized solution
 * - SOLVE: Enable solving state machine
 * - SOLVED: Wait for restart or reset
 * 
 * State Transition Diagram:
 * 
 *   IDLE_MAIN --[START]--> MAP --[END_MAP]--> READY
 *                           ^                    |
 *                           |                    |
 *                       [RESET]              [START]
 *                           |                    |
 *                           |                    v
 *   IDLE_MAIN <--[RESET]-- SOLVED <--[END_SOLVE]-- SOLVE
 *                           ^                        |
 *                           +-------[START]----------+
 * 
 * @note Called every loop iteration from main.cpp
 */
void main_FSM_handler() 
{
  // ========================================
  // STATE TRANSITIONS - Check conditions
  // ========================================
  switch (currentStateMain) 
  {
    case IDLE_MAIN:
      #ifdef DEBUG
      Serial.printf("-- Current state main = IDLE\n");
      #endif
      
      // Start mapping when user presses START button
      if (digitalRead(START_BUTTON)) {
        currentStateMain = MAP;
      }
      break;
    
    case MAP:
      #ifdef DEBUG
      Serial.printf("-- Current state main = MAP\n");
      #endif
      
      // Transition to READY when mapping completes
      if (END_MAP) {
        currentStateMain = READY;
      }
      break;
    
    case READY:
      #ifdef DEBUG
      Serial.printf("-- Current state main = READY\n");
      #endif
      
      // Start solving when user presses START button
      if (digitalRead(START_BUTTON)) {
        currentStateMain = SOLVE;
      }
      
      // Allow user to remap if needed
      if (digitalRead(RESET_BUTTON)) {
        currentStateMain = MAP;
      }
      break;
    
    case SOLVE:
      #ifdef DEBUG
      Serial.printf("-- Current state main = SOLVE\n");
      #endif
      
      // Transition to SOLVED when solution completes
      if (END_SOLVE) {
        currentStateMain = SOLVED;
      }
      break;
    
    case SOLVED:
      #ifdef DEBUG
      Serial.printf("-- Current state main = SOLVED\n");
      #endif
      
      // Allow re-running the solution
      if (digitalRead(START_BUTTON)) {
        currentStateMain = SOLVE;
      }
      
      // Full reset to beginning
      if (digitalRead(RESET_BUTTON)) {
        currentStateMain = IDLE_MAIN;
      }
      break;
  }
  
  // ========================================
  // STATE ACTIONS - Perform current state tasks
  // ========================================
  switch(currentStateMain)
  {
    case IDLE_MAIN:
      // TODO: Stop motors, clear path stacks
      break;

    case MAP:
      // Mapping is handled by map_FSM_handler()
      break;

    case READY:
      // TODO: Copy path_taken to a temporary stack for re-running
      // TODO: Display solution on screen/LEDs if available
      break;

    case SOLVE:
      // Solving is handled by solve_FSM_handler()
      break;

    case SOLVED:
      // Robot should be stopped
      break;

    default:
      break;
  }
}


// ============================================================================
// MAP STATE MACHINE - Maze Exploration
// ============================================================================

/**
 * @brief Mapping state machine handler
 * 
 * Implements wall-following algorithm to explore the maze:
 * 1. Follow line using PID control
 * 2. Detect junctions (L, R, T, Cross, Dead End)
 * 3. Make turning decisions based on wall-following rules
 * 4. Record path taken
 * 
 * Junction Detection Patterns (see IRLine.cpp for details):
 * +--------+----------+---------------------------+--------+
 * | Code   | Pattern  | Meaning                   | Action |
 * +--------+----------+---------------------------+--------+
 * | 'W'    | OOOOO    | Dead end (all white)      | U-turn |
 * | 'L'    | XXXOO    | Left junction             | Left   |
 * | 'R'    | OOXXX    | Right junction            | Fwd    |
 * | 'B'    | XXXXX    | T/Cross/End (all black)   | Fwd    |
 * | 'N'    | OOXOO    | Normal line               | Follow |
 * | 'E'    | Other    | Error/unstable            | Ignore |
 * +--------+----------+---------------------------+--------+
 * 
 * Wall Following Strategy:
 * - Always prefer left turns (left wall following)
 * - Move forward when right junction or cross detected
 * - U-turn at dead ends
 * 
 * @note Called every loop iteration from main.cpp when in MAP mode
 * @note Path recording is TODO - movements need to be pushed to path_taken stack
 */
void map_FSM_handler()
{
  // ========================================
  // STATE TRANSITIONS - Check conditions
  // ========================================
  switch(currentStateMap)
  {
    case IDLE_MAP:
      #ifdef DEBUG
      Serial.printf("-- Current state map = IDLE\n");
      #endif
      
      // Activate when main FSM enters MAP mode
      if (currentStateMain == MAP) {
        currentStateMap = FOLLOW_LINE_MAP;
      }
      break;

    case FOLLOW_LINE_MAP:
      #ifdef DEBUG
      Serial.printf("-- Current state map = FOLLOW_LINE\n");
      #endif
      
      // Detect junction type and transition accordingly
      char detectedNode = robot.IRLine.detectNode();
      
      if (detectedNode == 'W') {  // Dead end (all white) - OOOOO
        currentStateMap = U_TURN;
        // TODO: path_taken.push('U');
      }
      else if (detectedNode == 'R' || detectedNode == 'B') {  // Right or both
        // OOXXX or XXXXX - need to move forward to determine type
        currentStateMap = SMALL_FORWARD;
        forwardStartTime = 0;  // Reset forward timer
      }
      else if (detectedNode == 'L') {  // Left junction - XXXOO
        currentStateMap = LEFT_TURN_MAP;
        // TODO: path_taken.push('L');
      }
      break;

    case U_TURN:
      #ifdef DEBUG
      Serial.printf("-- Current state map = U_TURN\n");
      #endif
      
      // Wait for turn completion (time-based, see robot.cpp)
      if (robot.END_TURN) {
        currentStateMap = FOLLOW_LINE_MAP;
        robot.END_TURN = false;  // Reset flag for next turn
      }
      break;

    case LEFT_TURN_MAP:
      #ifdef DEBUG
      Serial.printf("-- Current state map = LEFT_TURN\n");
      #endif
      
      // Wait for turn completion
      if (robot.END_TURN) {
        currentStateMap = FOLLOW_LINE_MAP;
        robot.END_TURN = false;
      }
      break;

    case RIGHT_TURN_MAP:
      #ifdef DEBUG
      Serial.printf("-- Current state map = RIGHT_TURN\n");
      #endif
      
      // Wait for turn completion
      if (robot.END_TURN) {
        currentStateMap = FOLLOW_LINE_MAP;
        robot.END_TURN = false;
        // TODO: path_taken.push('R');
      }
      break;

    case SMALL_FORWARD:
      #ifdef DEBUG
      Serial.printf("-- Current state map = SMALL_FORWARD\n");
      #endif
      
      // Move forward briefly (200ms) to get past junction
      // This helps determine if it's a right turn, cross, or end
      
      char nodeAfterForward = robot.IRLine.detectNode();
      
      if (nodeAfterForward == 'N') {  // Back on normal line - OOXOO
        currentStateMap = FOLLOW_LINE_MAP;
        
        // ISSUE: This references test FSM state instead of map FSM state
        // Should use REVERSE instead of BACKWARD_TEST
        if (cross) {
          cross = false;
          currentStateMap = BACKWARD_TEST;  // FIXME: Should be REVERSE
        }
      }
      else if (nodeAfterForward == 'B') {  // Still all black = end - XXXXX
        currentStateMap = END;
      }
      break;

    case FORWARD_MAP:
      #ifdef DEBUG
      Serial.printf("-- Current state map = FORWARD\n");
      #endif
      
      // ISSUE: This state seems unused - always transitions immediately
      // TODO: Clarify purpose or remove this state
      if (1) {
        currentStateMap = FOLLOW_LINE_MAP;
      }
      break;

    case END:
      #ifdef DEBUG
      Serial.printf("-- Current state map = END\n");
      #endif
      
      // Mapping complete - prepare solution
      if (1) {
        END_MAP = true;
        currentStateMap = IDLE_MAP;
        
        // TODO: Optimize path before solving
        // solved_path = get_path(path_taken);
        
        #ifdef DEBUG
        Serial.println("Mapping complete!");
        // Print taken path for debugging
        // while (!path_taken.empty()) {
        //   Serial.print(path_taken.top());
        //   path_taken.pop();
        // }
        #endif
      }
      break;
  }

  // ========================================
  // STATE ACTIONS - Perform current state tasks
  // ========================================
  switch(currentStateMap)
  {
    case IDLE_MAP:
      robot.stop();
      break;

    case FOLLOW_LINE_MAP:
      // PID-controlled line following
      // See robot.cpp for PID implementation
      robot.followLine();
      break;

    case U_TURN:
      // Rotate 180° using timing
      // Duration: 80ms (seems very short - may need calibration)
      robot.u_turn();
      break;

    case LEFT_TURN_MAP:
      // Rotate 90° left using timing
      // Duration: 800ms
      robot.left_turn();
      break;

    case RIGHT_TURN_MAP:
      // Rotate 90° right using timing
      // Duration: 800ms
      robot.right_turn();
      break;

    case REVERSE:
      // Move backwards at nominal speed
      // Currently unused in state transitions
      robot.reverse();
      break;

    case SMALL_FORWARD:
      // Move forward at nominal speed briefly
      // Used to clear junctions and determine junction type
      robot.forward();
      break;

    case FORWARD_MAP:
      // Move forward until next junction
      // Currently unused in practice
      robot.forward();
      break;

    case END:
      // Stop and prepare solution
      robot.stop();
      break;

    default:
      break;
  }
}


// ============================================================================
// SOLVE STATE MACHINE - Solution Execution
// ============================================================================

#ifdef SOLVE
/**
 * @brief Solving state machine handler
 * 
 * Executes the optimized solution path:
 * 1. Follow line until junction
 * 2. Pop next instruction from solution stack
 * 3. Execute turn or forward command
 * 4. Repeat until stack is empty
 * 
 * Instructions:
 * - 'L': Turn left at junction
 * - 'R': Turn right at junction
 * - 'F': Go straight through junction
 * - 'U': U-turn (shouldn't appear in optimized path)
 * 
 * @note Currently not called from main loop - needs integration
 * @note Variables like 'instruction', 'Detect_node', 'back_on_line' undefined
 * @todo Complete implementation and integrate into main loop
 */
void solve_FSM_handler()
{
  // ========================================
  // STATE TRANSITIONS
  // ========================================
  switch (currentStateSolve)
  {
    case IDLE_SOLVE:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = IDLE\n");
      #endif
      
      // Activate when main FSM enters SOLVE mode
      if (currentStateMain == SOLVE) {
        currentStateSolve = FOLLOW_LINE_SOLVE;
      }
      break;

    case FOLLOW_LINE_SOLVE:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = FOLLOW_LINE\n");
      #endif
      
      // TODO: Define Detect_node condition
      // Should detect when robot reaches a junction
      // Could use: robot.IRLine.detectNode() != 'N' && robot.IRLine.detectNode() != 'E'
      if (Detect_node) {
        currentStateSolve = GET_INSTRUCTION;
      }
      break;

    case GET_INSTRUCTION:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = GET_INSTRUCTION\n");
      #endif
      
      // Check if solution is complete
      if (solved_path.empty()) {
        currentStateSolve = FINISH;
      }
      else {
        // Pop next move from solution stack
        char instruction = solved_path.top();
        solved_path.pop();
        
        // Transition based on instruction
        if (instruction == 'R') {
          currentStateSolve = RIGHT_TURN_SOLVE;
        }
        else if (instruction == 'L') {
          currentStateSolve = LEFT_TURN_SOLVE;
        }
        else if (instruction == 'F') {
          currentStateSolve = FORWARD_SOLVE;
        }
      }
      break;

    case RIGHT_TURN_SOLVE:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = RIGHT_TURN_SOLVE\n");
      #endif
      
      // TODO: Define back_on_line condition
      // Could use: robot.IRLine.detectNode() == 'N'
      if (back_on_line) {
        currentStateSolve = FOLLOW_LINE_SOLVE;
      }
      break;

    case LEFT_TURN_SOLVE:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = LEFT_TURN_SOLVE\n");
      #endif
      
      // TODO: Define back_on_line condition
      if (back_on_line) {
        currentStateSolve = FOLLOW_LINE_SOLVE;
      }
      break;

    case FORWARD_SOLVE:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = FORWARD_SOLVE\n");
      #endif
      
      // TODO: Define OOXOO condition
      // Should detect when back on normal line
      // Could use: robot.IRLine.detectNode() == 'N'
      if (OOXOO) {
        currentStateSolve = FOLLOW_LINE_SOLVE;
      }
      break;

    case FINISH:
      #ifdef DEBUG
      Serial.printf("-- Current state solve = FINISH\n");
      #endif
      
      // Solution complete
      if (1) {
        END_SOLVE = true;
        currentStateSolve = IDLE_SOLVE;
      }
      break;
  }

  // ========================================
  // STATE ACTIONS
  // ========================================
  switch(currentStateSolve)
  {
    case IDLE_SOLVE:
      robot.stop();
      break;

    case FOLLOW_LINE_SOLVE:
      robot.followLine();
      break;

    case GET_INSTRUCTION:
      // Instruction popping happens in state transitions
      break;

    case RIGHT_TURN_SOLVE:
      robot.right_turn();
      break;

    case LEFT_TURN_SOLVE:
      robot.left_turn();
      break;

    case FORWARD_SOLVE:
      robot.forward();
      break;

    case FINISH:
      robot.stop();
      break;

    default:
      break;
  }
}
#endif // SOLVE

#endif // RUN


// ============================================================================
// SIMPLE TEST STATE MACHINE
// ============================================================================

/**
 * @brief Minimal state machine for basic testing
 * 
 * Just follows the line continuously, ignoring junctions.
 * Useful for:
 * - PID tuning
 * - Basic hardware testing
 * - Sensor calibration
 * 
 * @note Not called from main loop
 * @note "Fodase" is Portuguese slang, consider renaming to "BasicFollow"
 */
void FodaseFMSHandler()
{
  switch(currentStatefodase)
  {
    case FOLLOW_FODASE:
      robot.followLine();
      break;
  }
}


// ============================================================================
// TEST STATE MACHINE (Commented Out in Original)
// ============================================================================

/**
 * @brief Full test state machine implementation
 * 
 * This is similar to map_FSM_handler() but with additional features:
 * - Can pause/resume with START button
 * - More detailed debugging output
 * - Path recording and optimization testing
 * 
 * @note Currently commented out in the original control.cpp
 * @note Implementation spans lines ~360-650 in original control.cpp
 * @note Not included here to keep file size manageable
 */
void test_FSM_handler()
{
  // Implementation available in original control.cpp
  // Uncomment and adapt if needed for testing
}