/**
 * @file path_handler.h
 * @brief Header file for path optimization functions
 * 
 * This module provides path optimization for maze solving robots.
 * It takes a recorded path (with dead ends and backtracking) and
 * simplifies it by removing unnecessary movements using pattern matching.
 */

#ifndef GET_PATH_H
#define GET_PATH_H

#include <stack>
#include <Arduino.h>

using namespace std;

/**
 * @brief Optimizes the path by removing unnecessary movements.
 * 
 * This function processes a stack of movements recorded during maze
 * exploration and condenses it into an optimized path by recognizing
 * and eliminating patterns that represent dead ends or inefficient routes.
 * 
 * Movement Codes:
 * - 'L': Turn left at junction
 * - 'R': Turn right at junction
 * - 'F': Go forward through junction (straight)
 * - 'U': U-turn (180° turn, indicates dead end)
 * 
 * Pattern Matching Examples (Right Wall Following):
 * - RUR → F  (Right-U-turn-Right = should have gone straight)
 * - FUR → L  (Forward-U-turn-Right = should have turned left)
 * - RUF → L  (Right-U-turn-Forward = should have turned left)
 * - LUR → U  (Left-U-turn-Right = full 360°, simplify to U)
 * - RUL → U  (Right-U-turn-Left = full 360°, simplify to U)
 * 
 * The algorithm:
 * 1. Iteratively scans the path stack for 3-move patterns
 * 2. Replaces patterns with simplified moves
 * 3. Repeats until no more patterns found
 * 4. Returns the optimized path
 * 
 * Time Complexity: O(n²) worst case, where n is path length
 * Space Complexity: O(n) for temporary stack storage
 * 
 * @param movements_made Stack of recorded movements (L, R, F, U)
 * @return Stack containing the optimized path
 * 
 * @note The function does not modify the input stack
 * @note Optimization is based on wall-following algorithm (left or right)
 * @note Configure wall-following direction in path_handler.cpp
 */
stack<char> get_path(stack<char> movements_made);

/**
 * @brief Saves a move made by the robot to a stack.
 * 
 * This function pushes the given move character onto the stack of movements.
 * Called during maze exploration to record the path taken.
 * 
 * Move Types:
 * - 'L': Left turn executed
 * - 'R': Right turn executed
 * - 'F': Forward through junction
 * - 'U': U-turn (dead end)
 * 
 * @param move The move character to be saved (L, R, F, or U)
 * @param movements_made Stack to store the move in
 * 
 * @note WARNING: The current implementation takes movements_made by VALUE,
 *       not by reference. This means changes won't persist!
 * @todo Change signature to: void save_move(char move, stack<char>& movements_made)
 *       The '&' makes it a reference so modifications are saved.
 */
void save_move(char move, stack<char> movements_made);

/**
 * @brief Print the recorded path to Serial
 * 
 * @note Currently commented out in implementation
 * @note Was intended for debugging path recording
 */
void print_path();

#endif // GET_PATH_H