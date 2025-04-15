#include "main.h"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// Clamp Pistons
extern pros::adi::DigitalOut clampPistonL;
extern pros::adi::DigitalOut clampPistonR;
extern pros::adi::DigitalOut doinkerPiston;
extern pros::adi::DigitalOut doinkerPiston2;
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
extern bool doinkerP2;
extern int autonSelected;

extern int intake_motor_p;
extern int ladybrown_motor_p;
extern int intake_color_p;

// Drive hold for auton and driver control
extern bool driveHold;
extern bool intakeActive;
// Global flag to inhibit intake commands when a wrong-colored ring is
// encountered
extern bool intakeOverride;
// Global alliance selection variable; set true for red, false for blue.
extern bool isRedAlliance;

// Global flag to inhibit unstuck commands while a reversal is in progress.
extern bool unstuckOverride;

extern bool moveIntake(int state);
extern void moveIntakeOP(int state);
extern void clamp();
extern void ladyBrown(int state);

void blueSolo();
void blueSixRing();
void blueGoalRush();
void blueAlliance();
void redSolo();
void redSixRing();
void redGoalRush();
void redAlliance();
void skills();
void skills2();
