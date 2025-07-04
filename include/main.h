/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

// Includes
//#include "okapi/api.hpp"

// C++20 Standard library includes

// Vectors to contain data
#include <vector>
// Strings for better char arrays
#include <string>
// Intializer lists
#include <initializer_list>
// Streams for better IO
#include <sstream>
// Memory management with smart pointers
#include <memory>
// Perfect forwarding; pairs
#include <utility>
// Function args
#include <functional>
// Types
#include <type_traits>
// Math
#include <cmath>
// Clamping
#include <algorithm>
// Error numbers
#include <cerrno>
// Standard library
#include <cstdlib>
// Integers
#include <cstdint>
// Mapping dictionary
#include <map>
// Numbers
#include <numeric>
// Execution
#include <execution>
// Formatting
#include <format>
// Array
#include <array>
// Tuple
#include <tuple>
// Queue
#include <queue>
// Stack
#include <stack>
// List
#include <list>
// Unordered map
#include <unordered_map>
// Type indexes and IDs
#include <typeindex>

// Third party includes

// nlohmann/json for JSON parsing
#include "nlohmann/json.hpp"

// TODO: Refactor our code into different files--main.cpp is getting too big

// Using declarations to shorten common types and integers
using std::vector;
using std::string;
using std::map;

using std::unique_ptr;
using std::shared_ptr;

using std::int8_t;
using std::int16_t;
using std::int32_t;

// Shorthands for port types & other type aliases
using DigiPort = std::int8_t;
using MGPorts = vector<DigiPort>;
using AnalogPort = char;

using VoidFunc = std::function<void()>;
using VoidFuncVector = vector<VoidFunc>;

using nlohmann::json;  // JSON library for parsing and serialization

// Options file with ports, config, etc
#include "options.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
