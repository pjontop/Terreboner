#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::Motor intake(-20);
inline pros::Motor outake(20);   // Intake motor on port 20
inline pros::Motor lever(-14);   // Outake motor on port 14

inline ez::Piston matchload('B');  // Dropper piston on ADI port B
inline ez::Piston forebar('A');      // Arm piston on ADI port A
inline ez::Piston descore_wing('H');  // Descore wing on ADI port H