#pragma once

#include "EZ-Template/api.hpp"

//
// CONSTANTS
//

void default_constants();

//
// TUNING
//

void drive_test(int inches);
void turn_test(int degrees);
void swing_test(int degrees);
void heading_test(int degrees);
void constants_test();

//
// RIGHT AUTONS
//

void right_split();
void right_greed();
void right_awp();

//
// LEFT AUTONS
//

void left_split();
void left_greed();
void left_awp();

//
// SKILLS
//

void skills();