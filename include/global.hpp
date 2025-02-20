#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "main.h"

// Clamp Pistons
extern pros::adi::DigitalOut clampPistonL;
extern pros::adi::DigitalOut clampPistonR;
extern pros::adi::DigitalOut doinkerPiston;
extern pros::adi::DigitalOut intakePiston;

// Intake Motor
extern pros::Motor intakeMotor;

// Lady Brown Motors
extern pros::Motor ladyBrownMotor;

// Lady Brown Rotational Sensor
extern pros::Rotation LadyBrownRotation;

// Color Sensor
extern pros::Optical IntakeColor;

// Constants
extern int ladybrown_counter;

// State variables
extern bool clampP1, clampP2;
extern bool intakeP;
extern bool doinkerP;
extern int autonSelected;

extern int intake_motor_p;
extern int ladybrown_motor_p;
extern int intake_color_p;

// Drive hold for auton and driver control
extern bool driveHold;

extern void moveIntake(int state);
extern void moveIntakeOP(int state);
extern void clamp();
extern void ladyBrown(int state);

void blueSolo();
void blueSixRing();
void blueGoalRush();
void redSolo();
void redSixRing();
void redGoalRush();
void skills();