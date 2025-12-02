/**
 * @file path_handler.cpp
 * @brief Handles the robot's path logic and optimization
 * 
 * This module implements path optimization for maze-solving robots.
 * It removes unnecessary movements (dead ends, redundant turns) by
 * recognizing patterns in the recorded path.
 */

#include <stack>
#include <string>
#include <Arduino.h>

//#define FOLLOW LEFT   // Enable for left wall following
#define FOLLOW RIGHT    // Enable for right wall following
//#define DEBUG

using namespace std;


/**
 * @brief Optimizes the robot's movement path by reducing unnecessary turns.
 * 
 * This function processes a stack of movements and simplifies it by
 * recognizing patterns that represent dead ends or inefficient routes.
 * 
 * Algorithm Overview:
 * 1. Transfer moves from input stack to temporary stack
 * 2. While transferring, check for 3-move patterns
 * 3. Replace patterns with optimized moves
 * 4. Repeat until no more patterns found (changed = false)
 * 5. Reverse final stack to restore original order
 * 
 * Why Patterns Work:
 * Consider a robot following the right wall that encounters a dead end:
 * - Goes right (R)
 * - Hits dead end, U-turns (U)
 * - Goes right again (R)
 * - Total: RUR
 * - Optimal: Should have gone straight (F)
 * 
 * The patterns represent all possible dead-end scenarios for
 * wall-following algorithms.
 * 
 * Wall Following Configuration:
 * - Right wall: Robot keeps right side near wall
 * - Left wall: Robot keeps left side near wall
 * - Patterns differ based on which wall is followed
 * 
 * @param movements_made Stack of characters representing the robot's movements:
 *                       'L' = Left turn
 *                       'R' = Right turn
 *                       'U' = U-turn (180°)
 *                       'F' = Forward (straight through junction)
 * 
 * @return A stack of characters representing the optimized path
 * 
 * @note Input stack is passed by value (copied), so original is preserved
 * @note Time complexity: O(n²) where n = path length (due to multiple passes)
 * @note Space complexity: O(n) for temporary storage
 */
stack<char> get_path(stack<char> movements_made) 
{
  stack<char> tempStack;  // Temporary storage for pattern matching
  bool changed = true;    // Flag indicating if optimization occurred
  
  // Keep optimizing until no more patterns are found
  while (changed) {
    changed = false;
    
    // Transfer moves from input to temp stack, checking for patterns
    while (!movements_made.empty()) {
      char top = movements_made.top();
      movements_made.pop();
      tempStack.push(top);
      
      // Need at least 3 elements to form a pattern
      if (tempStack.size() >= 3) {
        // Extract top three elements for pattern matching
        char third = tempStack.top(); tempStack.pop();
        char second = tempStack.top(); tempStack.pop();
        char first = tempStack.top(); tempStack.pop();
        
        // Construct pattern string for matching
        string pattern = {first, second, third};
        
        #ifdef FOLLOW LEFT
        /**
         * LEFT WALL FOLLOWING PATTERNS
         * 
         * In left wall following, the robot prefers left turns.
         * These patterns represent situations where the robot
         * went into a dead end and backtracked.
         */
        
        if (pattern == "LUL") {
          // Left-U-turn-Left → should have gone straight
          tempStack.push('F');
          changed = true;
        } 
        else if (pattern == "FUL") {
          // Forward-U-turn-Left → should have turned right
          tempStack.push('R');
          changed = true;
        } 
        else if (pattern == "LUF") {
          // Left-U-turn-Forward → should have turned right
          tempStack.push('R');
          changed = true;
        } 
        else if (pattern == "RUL") {
          // Right-U-turn-Left → 360° rotation → simplify to U-turn
          tempStack.push('U');
          changed = true;
        } 
        else if (pattern == "LUR") {
          // Left-U-turn-Right → 360° rotation → simplify to U-turn
          tempStack.push('U');
          changed = true;
        } 
        else {
          // No pattern match, restore original moves
          tempStack.push(first);
          tempStack.push(second);
          tempStack.push(third);
        }
        #endif // FOLLOW LEFT

        #ifdef FOLLOW RIGHT
        /**
         * RIGHT WALL FOLLOWING PATTERNS
         * 
         * In right wall following, the robot prefers right turns.
         * These patterns represent dead-end backtracking scenarios.
         * 
         * Pattern Logic:
         * - RUR: Right, hit dead end, came back right → was a straight path
         * - FUR: Went forward, dead end, came back right → should've gone left
         * - RUF: Went right, dead end, came back forward → should've gone left
         * - LUR/RUL: Opposite turns around U-turn → full circle → still a U-turn
         */
        
        if (pattern == "RUR") {
          // Right-U-turn-Right → should have gone Forward
          // Example: R→|  →U  →R  =  F→
          tempStack.push('F');
          changed = true;
        } 
        else if (pattern == "FUR") {
          // Forward-U-turn-Right → should have turned Left
          // Example: →| →U  →R  =  ←
          tempStack.push('L');
          changed = true;
        } 
        else if (pattern == "RUF") {
          // Right-U-turn-Forward → should have turned Left
          // Example: R→|  →U  →  =  ←
          tempStack.push('L');
          changed = true;
        } 
        else if (pattern == "LUR") {
          // Left-U-turn-Right → 360° rotation → simplify to U-turn
          // This represents going around a complete loop
          tempStack.push('U');
          changed = true;
        } 
        else if (pattern == "RUL") {
          // Right-U-turn-Left → 360° rotation → simplify to U-turn
          // This represents going around a complete loop
          tempStack.push('U');
          changed = true;
        } 
        else {
          // No pattern match, restore original three moves
          tempStack.push(first);
          tempStack.push(second);
          tempStack.push(third);
        }
        #endif // FOLLOW RIGHT

      }
    }
    
    // Transfer back to original stack for next iteration
    // This also reverses the stack back to correct order
    while (!tempStack.empty()) {
      movements_made.push(tempStack.top());
      tempStack.pop();
    }
  }

  // Final reversal to restore original move order
  // (Stack operations reverse order, so we reverse again)
  stack<char> final_path;
  while (!movements_made.empty()) {
    final_path.push(movements_made.top());
    movements_made.pop();
  }

  #ifdef DEBUG
  // Print the optimized path for debugging
  Serial.print("Final Path: ");
  stack<char> debug_copy;
  
  // Print and save to debug_copy to preserve final_path
  while (!final_path.empty()) {
    char c = final_path.top();
    Serial.print(c);
    debug_copy.push(c);
    final_path.pop();
  }
  Serial.println();
  
  // Restore final_path from debug_copy
  while (!debug_copy.empty()) {
    final_path.push(debug_copy.top());
    debug_copy.pop();
  }
  #endif

  // Return the final optimized path
  return final_path;
}

/**
 * @brief Save a move made by the robot to the path stack
 * 
 * This function should be called after each navigation decision
 * during maze exploration to record the path taken.
 * 
 * @param move The move character to save (L, R, F, or U)
 * @param movements_made Stack to store the movement in
 * 
 * @warning CRITICAL BUG: This function takes movements_made by VALUE,
 *          not by REFERENCE. This means the stack is COPIED when the
 *          function is called, and changes to the copy are LOST when
 *          the function returns.
 * 
 * @todo FIX: Change signature to:
 *       void save_move(char move, stack<char>& movements_made)
 *       
 *       The '&' makes it a reference parameter, so modifications
 *       will persist after the function returns.
 * 
 * Example of the bug:
 * ```
 * stack<char> path;
 * save_move('L', path);  // This does NOTHING!
 * // path is still empty because save_move modified a COPY
 * ```
 * 
 * Correct implementation:
 * ```
 * void save_move(char move, stack<char>& movements_made) {
 *   movements_made.push(move);
 * }
 * ```
 */
void save_move(char move, stack<char> movements_made)
{
  // Save the move to the stack
  // NOTE: This only modifies the local copy!
  movements_made.push(move);
  
  #ifdef DEBUG
  Serial.print("Move saved: ");
  Serial.println(move);
  #endif
}

// Commented out from original code:
// This function was meant to print the recorded path for debugging
//
// void print_path()
// {
//   // Print the path from a copy of movements_made
//   stack<char> tempStack = movements_made;  // Make a temporary copy
//   Serial.print("Path taken: ");
//   while (!tempStack.empty()) {
//     char c = tempStack.top();
//     Serial.print(c);
//     tempStack.pop();
//   }
//   Serial.println();
// }