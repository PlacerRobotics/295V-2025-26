#include "main.h"
#include "global.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-9, -8, -7}, pros::MotorGearset::blue,
                            pros::v5::MotorUnits::degrees);

pros::MotorGroup rightMotors({12, 13, 14}, pros::MotorGearset::blue,
                             pros::v5::MotorUnits::degrees);

// Inertial Sensor on port 10
pros::Imu imu(1);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-11);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(21);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot
// (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2,
                                 -0.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot
// (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    14,                         // 14 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25 omnis
    450,                        // drivetrain rpm is 450
    8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
    linearController(7,   // proportional gain (kP) // 6.5
                     0,   // integral gain (kI)
                     10,  // derivative gain (kD) // 30
                     0.5, // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(1.6, // proportional gain (kP)
                      0,   // integral gain (kI)
                      11,  // derivative gain (kD)
                      1,   // anti windup
                      1,   // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr,   // vertical tracking wheel 2, set to
                                       // nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr,     // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  1.019 // expo curve gain 1.019
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

/*


Personal Code


*/
// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(example_txt); // '.' replaced with "_" to make c++ happy

// Third Stake Path
ASSET(thirdStake_txt);
ASSET(thirdStake2_txt);
// Fourth Stake Path
ASSET(fourthStake_txt);

// Second Ring Path
ASSET(ladyBrownRing1_txt);

// Fourth Ring Path
ASSET(ladyBrownRing2_txt);

bool clampP1 = false;
bool clampP2 = false;
bool intakeP = false;
bool doinkerP = false;
bool doinkerP2 = false;

bool driveHold = false;

// Motors
int intake_motor_p = 10;
int ladybrown_motor_p = 6;
int intake_color_p = 20;

// Intake Motor
pros::Motor intakeMotor(intake_motor_p, pros::v5::MotorGears::blue,
                        pros::v5::MotorUnits::degrees);

// Lady Brown Motors
pros::Motor ladyBrownMotor(ladybrown_motor_p, pros::v5::MotorGears::red,
                           pros::v5::MotorUnits::degrees);

// Lady Brown Rotational Sensor
pros::Rotation LadyBrownRotation(2);
// Color Sensor
pros::Optical IntakeColor(intake_color_p);

// Lady Brown Counter
int ladybrown_counter = 0;

// State variable for intake
bool intakeActive = false;

// Global flag to inhibit intake commands when a wrong-colored ring is
// encountered
bool intakeOverride = false;

// Global flag to inhibit unstuck commands while a reversal is in progress.
bool unstuckOverride = false;

// Global alliance selection variable; set true for red, false for blue.
bool isRedAlliance = true;

bool moveIntake(int state) {
  // Use voltage values within the V5 limits.
  const int voltages[3] = {0, 127 * 120, -127 * 120};
  if (state < 0 || state > 2) {
    intakeActive = false;
    return false;
  }
  intakeMotor.move_voltage(voltages[state]);

  // Return true if state is 1 or 2, false if 0.
  intakeActive = (state == 1 || state == 2);
  return intakeActive;
}

void moveIntakeOP(int state) {
  // Full voltage commands for off, positive, and negative directions
  const int voltages[3] = {0, 120 * 127, -120 * 127};
  if (state < 0 || state > 2) {
    return;
  }
  // Command the motor with the desired voltage.
  intakeMotor.move_voltage(voltages[state]);
}

void clamp() {
  clampP1 = !clampP1;
  clampP2 = !clampP2;
  pros::delay(150); // Ensure pneumatic consistency
}

// Pistons

int clampPistonl_p = 'A';
int clampPistonr_p = 'C';
int intake_piston_p = 'D';
int doinker_piston_p = 'B';
int doinker_piston2_p = 'E';

// Clamp Pistons
pros::adi::DigitalOut clampPistonL(clampPistonl_p);
pros::adi::DigitalOut clampPistonR(clampPistonr_p);

// Intake Pistons
pros::adi::DigitalOut intakePiston(intake_piston_p);

// Doinker Pistons
pros::adi::DigitalOut doinkerPiston(doinker_piston_p);
pros::adi::DigitalOut doinkerPiston2(doinker_piston2_p);
// Define a structure to pass parameters to the PID task
struct PIDParams {
  volatile double
      targetOutput; // volatile, as it may be updated from another task
  bool active;      // flag to enable/disable the PID control
};

// Global pointer to our PID parameters (so we can update without starting a new
// task)
PIDParams *liftPIDParams = nullptr;

double normalizeAngleError(double error) {
  while (error > 180)
    error -= 360;
  while (error < -180)
    error += 360;
  return error;
}
// PID task function using only the P term.
void liftControlPIDTask(void *param) {
  PIDParams *pid = static_cast<PIDParams *>(param);

  // Retuned parameters for smoother control on a single 11W motor (~100 rpm
  // max) Lower gain, higher tolerance and deadband, and slower update frequency
  // can help reduce oscillation.
  const double kp = 0.6; // Lower proportional gain
  const double tolerance =
      1.0; // Allowable error (degrees) before we're "close enough"
  const double deadband =
      7.0;                   // If error is smaller than 10°, command no motion
  const int minVelocity = 5; // Minimum velocity (rpm) when commanding movement
  const int maxVelocity = 60; // Reduced maximum velocity for smoother response
  const int loopDelay = 50; // Increase the delay to 50 ms (slower update rate)

  // Use a more aggressive low-pass filter to smooth the sensor reading.
  double filteredOutput = LadyBrownRotation.get_position() / 100.0;
  const double alpha =
      0.8; // Higher smoothing factor (0–1), closer to 1 means more filtering

  while (true) {
    if (!pid->active) {
      pros::delay(loopDelay);
      continue;
    }

    // Get and filter the sensor reading
    double rawOutput = LadyBrownRotation.get_position() / 100.0;
    filteredOutput = alpha * rawOutput + (1 - alpha) * filteredOutput;
    double error = pid->targetOutput - filteredOutput;

    // If error is within our tolerance, stop the motor.
    if (fabs(error) < tolerance) {
      ladyBrownMotor.move_velocity(0);
    } else {
      double velocityCommand = kp * error;

      // Zero output if error is within the deadband (ignoring small
      // fluctuations)
      if (fabs(error) < deadband) {
        velocityCommand = 0;
      } else if (fabs(velocityCommand) < minVelocity) {
        velocityCommand = (velocityCommand > 0) ? minVelocity : -minVelocity;
      }
      if (velocityCommand > maxVelocity) {
        velocityCommand = maxVelocity;
      } else if (velocityCommand < -maxVelocity) {
        velocityCommand = -maxVelocity;
      }

      ladyBrownMotor.move_velocity(velocityCommand);
    }

    pros::delay(loopDelay);
  }
}
// Global helper functions for lift control

// Start the continuous PID task (call this once, e.g., during initialization)
void startContinuousLiftControl() {
  if (liftPIDParams == nullptr) {
    liftPIDParams = new PIDParams;
    // Initialize the target with the current lift reading
    liftPIDParams->targetOutput = LadyBrownRotation.get_position() / 100.0;
    liftPIDParams->active = true;
    pros::Task liftPIDTask(liftControlPIDTask, liftPIDParams);
  }
}

// Update the lift target without starting a new task
void updateLiftTarget(double newTarget) {
  if (liftPIDParams != nullptr) {
    liftPIDParams->targetOutput = newTarget;
  }
}

// To disable PID control:
void disableLiftControl() {
  if (liftPIDParams != nullptr) {
    liftPIDParams->active = false;
  }
}

// To enable PID control:
void enableLiftControl() {
  if (liftPIDParams != nullptr) {
    liftPIDParams->active = true;
  }
}

void ladyBrown(int state) {
  // const float positions[4] = {10, 45, 80, 150};
  const float positions[4] = {10, 38, 50, 170};
  if (state >= 0 && state < 4) {
    updateLiftTarget(positions[state]);
    // if (state == 3) {
    //   ladyBrownMotor.move_absolute(positions[state], 100);
    // } else {
    // ladyBrownMotor.move_absolute(positions[state], 50);
    // }
  }
}

void skills() {
  // Skills Autonomous

  // Scores on the alliance stake
  chassis.setPose(-61, 0, 90);
  // chassis.setPose(-64.5, 64.5, 135);

  // Movement 1
  // Moves forward to align with the stake
  moveIntake(1);
  pros::delay(500);
  moveIntake(0);
  chassis.moveToPoint(-48.5, 0, 300);
  chassis.turnToPoint(-48.5, -27, 500, {.forwards = false});
  chassis.moveToPoint(-48.5, -27, 800, {.forwards = false, .maxSpeed = 60},
                      false);
  // clamps the stake
  pros::delay(200);
  clamp();

  // Movement 2
  //   Moves to first ring
  chassis.turnToPoint(-23.5, -23.5, 600, {});
  moveIntake(0);
  pros::delay(500);
  moveIntake(1);
  chassis.moveToPoint(-23.5, -23.5, 800);
  pros::delay(500);

  // Movement 3
  // Moves to the second ring
  chassis.turnToPoint(-4, -56, 600);
  chassis.moveToPoint(-4, -56, 800, {}, false);
  //   chassis.turnToPoint(-18, -40, 600, {}, false);
  //   pros::delay(500);
  //   ladyBrown(1);
  //   chassis.follow(ladyBrownRing1_txt, 14, 1200);
  //   pros::delay(500);
  // Moves to second ring
  chassis.turnToPoint(24, -50, 800, {}, false);
  // Wall stake up
  chassis.moveToPoint(24, -50, 1000);
  pros::delay(300);
  ladyBrown(1);

  // Movement 4
  // Moves to middle
  chassis.turnToPoint(6, -50, 500, {.forwards = false});
  chassis.moveToPoint(6, -50, 800, {.forwards = false}, false);
  // Moves to third ring
  chassis.turnToHeading(180, 800, {}, false);
  moveIntake(0);
  //   ladyBrown(2);
  //   pros::delay(500);
  //   moveIntake(1);
  chassis.moveToPoint(6, -65, 800, {}, false);
  //   pros::delay(500);
  ladyBrown(3);
  pros::delay(500);

  // Movement 5
  // Moves to middle
  chassis.moveToPoint(0, -50, 600, {.forwards = false}, false);
  ladyBrown(0);
  // Moves to fourth ring
  chassis.turnToPoint(-23.5, -50, 800, {}, false);
  moveIntake(1);
  chassis.moveToPoint(-23.5, -50, 800, {}, false);

  // Movement 6 & 7
  chassis.moveToPoint(-59, -50, 2100, {.maxSpeed = 50}, false);
  pros::delay(500);

  // Movement 8
  chassis.turnToPoint(-50, -61, 800);
  chassis.moveToPoint(-50, -61, 1400, {}, false);

  // Movement 9
  //   chassis.turnToPoint(-65, -65, 500, {.forwards = false});
  chassis.moveToPoint(-60, -66, 800, {.forwards = false}, false);
  pros::delay(200);
  clamp();

  /*
  Second Clamp Movements
  */

  // Movement 10
  chassis.moveToPoint(-46, -47.5, 800);
  moveIntake(0);
  chassis.turnToPoint(-46, 13, 800, {.forwards = false});
  chassis.moveToPoint(-46, 13, 800, {.forwards = false, .minSpeed = 127},
                      false);
  chassis.moveToPoint(-46, 26, 800, {.forwards = false, .maxSpeed = 50}, false);
  pros::delay(200);
  clamp();
  // Movement 11
  // Moves to first ring
  chassis.turnToPoint(-24, 24, 500, {}, false);
  moveIntake(1);
  chassis.moveToPoint(-24, 24, 700);
  pros::delay(500);

  // Movement 12
  // Moves to the center
  chassis.turnToPoint(-4, 55, 600);
  chassis.moveToPoint(-4, 55, 800, {}, false);
  chassis.turnToPoint(24, 50, 800, {}, false);
  // Wall stake up
  chassis.moveToPoint(24, 50, 1000);
  pros::delay(300);
  ladyBrown(1);

  // Movement 13
  // Moves to middle
  chassis.turnToPoint(6, 50, 500, {.forwards = false});
  chassis.moveToPoint(6, 50, 800, {.forwards = false}, false);
  // Moves to third ring
  chassis.turnToHeading(0, 800, {}, false);
  moveIntake(0);
  chassis.moveToPoint(6, 65, 800, {}, false);
  ladyBrown(3);
  pros::delay(800);

  // Reset position
  //   chassis.setPose(0, 62, 0);

  // Movement 14
  // Moves to middle
  chassis.moveToPoint(0, 50, 1000, {.forwards = false}, false);
  ladyBrown(0);
  // Moves to fourth ring
  chassis.turnToPoint(-23.5, 50, 800, {}, false);
  moveIntake(1);
  chassis.moveToPoint(-23.5, 50, 800, {}, false);

  // Movement 15 & 16
  chassis.moveToPoint(-59, 50, 2000, {.maxSpeed = 50}, false);
  pros::delay(500);

  // Movement 17
  chassis.turnToPoint(-50, 61, 800);
  chassis.moveToPoint(-50, 61, 1400, {}, false);

  // Movement 18
  //   chassis.turnToPoint(-65, 65, 500, {.forwards = false});
  chassis.moveToPoint(-60, 66, 800, {.forwards = false}, false);
  pros::delay(200);
  clamp();

  /*

  // Third Stake movements

  // */
  // Moves forward a bit
  chassis.moveToPoint(-54, 54, 500);
  //   chassis.turnToPoint(-54, 54, 1200);
  //   chassis.moveToPoint(-54, 54, 1500);
  moveIntake(0);

  // Movement 19
  // chassis.turnToPoint(23.5, 23.5, 800);
  // chassis.moveToPoint(23.5, 23.5, 1500);
  chassis.turnToPoint(-12, 51, 600);
  chassis.follow(thirdStake_txt, 14, 2000);
  pros::delay(1500);
  moveIntake(1);
  pros::delay(500);
  moveIntake(0);

  // Movement 20
  chassis.turnToPoint(52, -3, 800, {.forwards = false});
  chassis.moveToPoint(52, -3, 1200, {.forwards = false, .maxSpeed = 60}, false);
  pros::delay(200);
  clamp();
  pros::delay(300);
  moveIntake(1);

  // Movement 21
  chassis.turnToPoint(26, -26, 800);
  chassis.moveToPoint(26, -26, 1000, {}, false);
  pros::delay(500);

  // Movement 22
  chassis.turnToPoint(2.5, -2.5, 800, {}, false);
  moveIntake(0);
  chassis.moveToPoint(2.5, -2.5, 1200, {});
  pros::delay(500);
  moveIntake(1);
  pros::delay(600);
  moveIntake(0);

  // Movement 23
  chassis.turnToPoint(45, 45, 800, {}, false);
  chassis.moveToPoint(45, 45, 1500);
  pros::delay(600);
  moveIntake(1);

  // Movement 24
  //   chassis.turnToPoint(39, 39, 800, {.forwards = false});
  chassis.moveToPoint(39, 39, 600, {.forwards = false});

  chassis.turnToPoint(59.5, 45.5, 800);
  chassis.moveToPoint(59.5, 45.5, 800, {}, false);

  // Movement 25
  //   chassis.turnToPoint(39, 39, 600, {.forwards = false});
  chassis.moveToPoint(39, 39, 600, {.forwards = false});

  chassis.turnToPoint(47, 61, 800);
  chassis.moveToPoint(47, 61, 800);

  // Movement 26
  chassis.turnToPoint(63, 59, 600);
  doinkerP = !doinkerP;
  chassis.moveToPoint(63, 59, 800, {.minSpeed = 127});
  pros::delay(500);
  chassis.turnToHeading(
      -135, 800, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 127},
      false);
  // pros::delay(1000);
  moveIntake(0);
  chassis.moveToPoint(63, 63, 800, {.forwards = false, .minSpeed = 127}, false);
  clamp();
  pros::delay(50);
  doinkerP = !doinkerP;

  /*
  Fourth Stake Movements
  */

  // Movement 27
  chassis.moveToPoint(49, 49, 600, {.minSpeed = 127});
  chassis.turnToPoint(59, -21, 300, {.minSpeed = 127});
  chassis.moveToPoint(59, -21, 1000, {.minSpeed = 127});
  //   chassis.follow(fourthStake_txt, 14, 2000, false, false);

  // Movement 28
  // chassis.turnToPoint(59, -23.5, 600, {.forwards = false});
  // chassis.moveToPoint(59, -23.5, 1000, {.forwards = false}, false);

  // Movement 28
  chassis.moveToPoint(65, -65, 1500, {.minSpeed = 127}, false);
  chassis.moveToPoint(47.5, -47.5, 500, {.forwards = false, .minSpeed = 127});
}

void blueSolo() {
  // Blue Side Negative Corner Autonomous
  chassis.setPose(50, 23.5, 90);
  // Move to the stake
  chassis.moveToPoint(23.5, 23.5, 800, {.forwards = false}, false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1 Left
  chassis.turnToPoint(10, 41.5, 1000);
  chassis.moveToPoint(10, 41.5, 1000);
  // Ring 2 Right
  chassis.moveToPoint(8, 52, 800);
  // Back up
  chassis.turnToPoint(15, 38, 800, {.forwards = false});
  chassis.moveToPoint(15, 38, 800, {.forwards = false});
  // Ring 3 Middle
  chassis.turnToPoint(24, 47, 800);
  chassis.moveToPoint(24, 47, 800);
  pros::delay(200);

  // Ring 4 Middle
  chassis.turnToPoint(47.5, 12, 800, {}, false);
  moveIntake(0);
  chassis.moveToPoint(47.5, 12, 800);

  chassis.turnToPoint(47.5, -11, 600, {}, false);
  clamp();
  moveIntake(1);
  chassis.moveToPoint(47.5, -11, 2000, {.maxSpeed = 30}, false);
  moveIntake(0);

  // Stake 2
  chassis.turnToPoint(23.5, -23.5, 600, {.forwards = false});
  chassis.moveToPoint(23.5, -23.5, 800, {.forwards = false});
  pros::delay(200);
  clamp();
  moveIntake(1);

  // Ring 5 Middle
  chassis.turnToPoint(23.5, -47.5, 800);
  chassis.moveToPoint(23.5, -47.5, 800);
  pros::delay(200);

  // Touch ladder
  chassis.moveToPoint(23.5, -23.5, 800, {.forwards = false});
  chassis.turnToPoint(9, -23.5, 800, {}, false);
  moveIntake(0);
  chassis.moveToPoint(9, -23.5, 800);
}

void blueSixRing() {
  chassis.setPose(50, 23.5, 90);
  // Move to the stake
  chassis.moveToPoint(24, 23.5, 800, {.forwards = false}, false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1 Left
  chassis.turnToPoint(8, 41.5, 800);
  chassis.moveToPoint(8, 41.5, 800);
  // Ring 2 Right
  chassis.moveToPoint(8, 56, 800);
  // Back up
  // chassis.turnToPoint(-15, 38, 800, {.forwards = false});
  chassis.moveToPoint(15, 38, 800, {.forwards = false});
  // Ring 3 Middle
  chassis.turnToPoint(23.5, 47.5, 800);
  chassis.moveToPoint(23.5, 47.5, 800);
  pros::delay(200);
  chassis.turnToPoint(19, 32, 800, {.forwards = false});
  chassis.moveToPoint(19, 32, 800, {.forwards = false});
  chassis.turnToPoint(41.5, 5, 800, {}, false);
  intakeP = !intakeP;
  chassis.moveToPoint(41, 5, 1000, {}, false);
  intakeP = !intakeP;
}

void blueGoalRush() {
  // Blue Positive Side Rush
  chassis.setPose(60, -47.5, 270);
  // Rush to the stake
  chassis.moveToPoint(10, -43, 1200, {.minSpeed = 127});
  moveIntake(1);
  pros::delay(800);
  doinkerP = !doinkerP;
  pros::delay(200);
  moveIntake(0);
  chassis.moveToPoint(29, -34, 1500, {.forwards = false}, false);
  doinkerP = !doinkerP;
  pros::delay(200);
  chassis.turnToPoint(24, -60, 800, {.forwards = false});
  chassis.moveToPoint(24, -60, 800, {.forwards = false, .maxSpeed = 50}, false);
  //   pros::delay(500);
  clamp();
  moveIntake(1);
  pros::delay(800);
  moveIntake(0);
  clamp();
  ;
  chassis.turnToPoint(23.5, -23.5, 800, {.forwards = false});
  chassis.moveToPoint(23.5, -23.5, 1500, {.forwards = false, .maxSpeed = 60},
                      false);
  pros::delay(200);
  clamp();
  //   moveIntake(1);
  chassis.turnToPoint(55, -55, 600, {}, false);
  chassis.moveToPoint(55, -55, 1200);
  pros::delay(500);
  moveIntake(1);
  //   pros::delay(200);
  //   ladyBrown(3);
  //   chassis.moveToPoint(63.5, -63.5, 1000);
  //   chassis.moveToPoint(55, -55, 1200, {.forwards = false}, false);
  doinkerP = !doinkerP;
  chassis.moveToPoint(63, -59, 1000, {}, false);
  moveIntake(0);
  chassis.turnToHeading(-45, 1000,
                        {.direction = AngularDirection::CW_CLOCKWISE}, false);
  doinkerP = !doinkerP;
  chassis.turnToPoint(9, -26, 800);
  chassis.moveToPoint(9, -26, 3000, {}, false);
  ladyBrown(3);
}

void redSolo() {
  // Red Side Negative Corner Autonomous
  // chassis.setPose(-50, 23.5, 180);
  // Move to the stake
  chassis.setPose(-63, 10, 180);
  // chassis.turnToPoint(-66, 0, 800);
  // chassis.moveToPoint(-62, -12, 300);
  ladyBrown(3);
  pros::delay(500);
  ladyBrown(0);
  chassis.turnToPoint(-23.5, 23.5, 800, {.forwards = false});
  chassis.moveToPoint(-23.5, 23.5, 1200, {.forwards = false, .maxSpeed = 100}, false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1 Left
  chassis.turnToPoint(-10, 41.5, 1000);
  chassis.moveToPoint(-10, 41.5, 1000);
  // Ring 2 Right
  chassis.moveToPoint(-10, 56, 800);
  // Back up
  chassis.turnToPoint(-15, 38, 800, {.forwards = false});
  chassis.moveToPoint(-15, 38, 800, {.forwards = false});
  // Ring 3 Middle
  chassis.turnToPoint(-24, 47, 800);
  chassis.moveToPoint(-24, 47, 800);
  chassis.turnToPoint(-47.5, 0, 800, {}, false);
  // moveIntake(0);
  intakeP = !intakeP;
  chassis.moveToPoint(-47.5, 0, 1200, {}, false);
  moveIntake(0);
  chassis.turnToPoint(-23.5, 0, 800);
  chassis.moveToPoint(-23.5, 0, 800);
  intakeP = !intakeP;

  // Ring 4 Middle
  // chassis.turnToPoint(-47.5, 12, 800, {}, false);
  // moveIntake(0);
  // chassis.moveToPoint(-47.5, 12, 800);

  // chassis.turnToPoint(-47.5, -11, 600, {}, false);
  // clamp();
  // moveIntake(1);
  // chassis.moveToPoint(-47.5, -11, 2000, {.maxSpeed = 30}, false);
  // moveIntake(0);

  // // Stake 2
  // chassis.turnToPoint(-23.5, -23.5, 600, {.forwards = false});
  // chassis.moveToPoint(-23.5, -23.5, 800, {.forwards = false});
  // pros::delay(200);
  // clamp();
  // moveIntake(1);

  // // Ring 5 Middle
  // chassis.turnToPoint(-23.5, -47.5, 800);
  // chassis.moveToPoint(-23.5, -47.5, 800);
  // pros::delay(200);

  // // Touch ladder
  // chassis.moveToPoint(-23.5, -23.5, 800, {.forwards = false});
  // chassis.turnToPoint(-9, -23.5, 800, {}, false);
  // moveIntake(0);
  // chassis.moveToPoint(-9, -23.5, 800);
}

void redSixRing() {
  chassis.setPose(-50, 23.5, 270);
  // Move to the stake
  chassis.moveToPoint(-24, 23.5, 800, {.forwards = false}, false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1 Left
  chassis.turnToPoint(-8, 41.5, 800);
  chassis.moveToPoint(-8, 41.5, 800);
  // Ring 2 Right
  chassis.moveToPoint(-8, 56, 800);
  // Back up
  // chassis.turnToPoint(-15, 38, 800, {.forwards = false});
  chassis.moveToPoint(-15, 38, 800, {.forwards = false});
  // Ring 3 Middle
  chassis.turnToPoint(-23.5, 47.5, 800);
  chassis.moveToPoint(-23.5, 47.5, 800);
  pros::delay(200);
  chassis.turnToPoint(-19, 32, 800, {.forwards = false});
  chassis.moveToPoint(-19, 32, 800, {.forwards = false});
  chassis.turnToPoint(-41.5, 5, 800, {}, false);
  intakeP = !intakeP;
  chassis.moveToPoint(-41, 5, 1000, {}, false);
  intakeP = !intakeP;
}

void redGoalRush() {
  // Red Positive Side Rush
  chassis.setPose(-60, -47.5, 90);
  // Rush to the stake
  chassis.moveToPoint(-10, -53, 1500, {});
  moveIntake(1);
  pros::delay(800);
  doinkerP = !doinkerP;
  pros::delay(200);
  moveIntake(0);
  chassis.moveToPoint(-29, -34, 1500, {.forwards = false}, false);
  doinkerP = !doinkerP;
  pros::delay(200);
  chassis.turnToPoint(-24, -60, 800, {.forwards = false});
  chassis.moveToPoint(-24, -60, 800, {.forwards = false, .maxSpeed = 50},
                      false);
  //   pros::delay(500);
  clamp();
  moveIntake(1);
  pros::delay(800);
  moveIntake(0);
  clamp();
  ;
  chassis.turnToPoint(-23.5, -23.5, 800, {.forwards = false});
  chassis.moveToPoint(-23.5, -23.5, 1200, {.forwards = false, .maxSpeed = 50},
                      false);
  pros::delay(200);
  clamp();
  //   moveIntake(1);
  chassis.turnToPoint(-55, -55, 600, {}, false);
  chassis.moveToPoint(-55, -55, 1200);
  pros::delay(500);
  moveIntake(1);
  //   pros::delay(200);
  //   ladyBrown(3);
  //   chassis.moveToPoint(63.5, -63.5, 1000);
  //   chassis.moveToPoint(55, -55, 1200, {.forwards = false}, false);
  doinkerP = !doinkerP;
  chassis.moveToPoint(-63, -59, 1000, {}, false);
  moveIntake(0);
  chassis.turnToHeading(
      45, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  doinkerP = !doinkerP;
  chassis.turnToPoint(-9, -26, 800);
  chassis.moveToPoint(-9, -26, 3000, {}, false);
  ladyBrown(3);
}

void blueAlliance() {
  // Blue Alliance Wall Stake Autonomous
  chassis.setPose(56, 11, 0);
  chassis.moveToPoint(58, 0, 800, {.forwards = false});
  chassis.turnToHeading(270, 1500);
  chassis.moveToPoint(64, 0, 600, {.forwards = false}, false);
  moveIntake(1);
  pros::delay(1000);
  moveIntake(0);
  // Move to the stake
  //   chassis.turnToPoint(43.5, 30, 500, {.forwards = false});
  //   chassis.follow(blueNegative1_txt, 14, 1500, false, false);
  chassis.moveToPoint(58, 0, 600);
  chassis.turnToPoint(24, 23.5, 1000, {.forwards = false});
  chassis.moveToPoint(24, 23.5, 2000, {.forwards = false, .maxSpeed = 50},
                      false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1 Left
  chassis.turnToPoint(11, 41.5, 1000);
  chassis.moveToPoint(11, 41.5, 1000);
  // Ring 2 Right
  chassis.moveToPoint(11, 52, 600);
  // Back up
  chassis.turnToPoint(15, 38, 800, {.forwards = false});
  chassis.moveToPoint(15, 38, 800, {.forwards = false});
  // Ring 3 Middle
  chassis.turnToPoint(24, 47, 800);
  chassis.moveToPoint(24, 47, 800);
  pros::delay(200);

  // Starting ring 4
  //   chassis.turnToPoint(59, 0, 800);
  chassis.turnToPoint(21, 8, 800);
  chassis.moveToPoint(21, 8, 800, {}, false);
  ladyBrown(2);
}

void redAlliance() {
  // Blue Alliance Wall Stake Autonomous
  chassis.setPose(-56, 11, 0);
  chassis.moveToPoint(-58, 0, 800, {.forwards = false});
  chassis.turnToHeading(90, 1500);
  chassis.moveToPoint(-64, 0, 600, {.forwards = false}, false);
  moveIntake(1);
  pros::delay(1000);
  moveIntake(0);
  // Move to the stake
  //   chassis.turnToPoint(43.5, 30, 500, {.forwards = false});
  //   chassis.follow(blueNegative1_txt, 14, 1500, false, false);
  chassis.moveToPoint(-58, 0, 600);
  chassis.turnToPoint(-24, 23.5, 1000, {.forwards = false});
  chassis.moveToPoint(-24, 23.5, 2000, {.forwards = false, .maxSpeed = 50},
                      false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1 Left
  chassis.turnToPoint(-11, 41.5, 1000);
  chassis.moveToPoint(-11, 41.5, 1000);
  // Ring 2 Right
  chassis.moveToPoint(-11, 52, 600);
  // Back up
  chassis.turnToPoint(-15, 38, 800, {.forwards = false});
  chassis.moveToPoint(-15, 38, 800, {.forwards = false});
  // Ring 3 Middle
  chassis.turnToPoint(-24, 47, 800);
  chassis.moveToPoint(-24, 47, 800);
  pros::delay(200);

  // Starting ring 4
  //   chassis.turnToPoint(59, 0, 800);
  chassis.turnToPoint(-21, 8, 800);
  chassis.moveToPoint(-21, 8, 800, {}, false);
  ladyBrown(2);
}

void red2ring() {
  chassis.setPose(-50, -23.5, 270);
  chassis.moveToPoint(-23.5, -23.5, 1500, {.forwards = false, .maxSpeed = 60},
                      false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1
  chassis.turnToPoint(-23.5, -47.5, 800);
  chassis.moveToPoint(-23.5, -47.5, 1000);
  chassis.turnToPoint(-60, -60, 800, {}, false);
  chassis.moveToPoint(-60, -60, 1000);
  pros::delay(200);
  doinkerP = !doinkerP;
  pros::delay(400);
  moveIntake(0);
  chassis.turnToHeading(45, 1000, {.direction = AngularDirection::CW_CLOCKWISE},
                        false);
  pros::delay(200);
  doinkerP = !doinkerP;
  chassis.turnToPoint(-20, -50, 800);
  chassis.moveToPoint(-20, -50, 1000);
  pros::delay(500);
  clamp();
  chassis.turnToPoint(60, -60, 1000, {.forwards = false});
}

void blue2ring() {
  chassis.setPose(50, -23.5, 90);
  chassis.moveToPoint(23.5, -23.5, 1500, {.forwards = false, .maxSpeed = 60},
                      false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1
  chassis.turnToPoint(23.5, -47.5, 800);
  chassis.moveToPoint(23.5, -47.5, 1000);
  chassis.turnToPoint(61, -62, 800, {}, false);
  chassis.moveToPoint(61, -62, 1000);
  pros::delay(200);
  doinkerP2 = !doinkerP2;
  pros::delay(400);
  moveIntake(0);
  chassis.turnToHeading(
      -45, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  pros::delay(200);
  doinkerP2 = !doinkerP2;
  chassis.turnToPoint(20, -50, 800);
  chassis.moveToPoint(20, -50, 1000);
  pros::delay(500);
  clamp();
  chassis.turnToPoint(-60, -60, 1000, {.forwards = false});
}

void red2ringLadder() {
  chassis.setPose(-50, -23.5, 270);
  chassis.moveToPoint(-23.5, -23.5, 1500, {.forwards = false, .maxSpeed = 60},
                      false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1
  chassis.turnToPoint(-23.5, -47.5, 800);
  chassis.moveToPoint(-23.5, -47.5, 1000);
  chassis.turnToPoint(-60, -60, 800, {}, false);
  chassis.moveToPoint(-60, -60, 1000);
  pros::delay(200);
  doinkerP = !doinkerP;
  pros::delay(400);
  moveIntake(0);
  chassis.turnToHeading(45, 1000, {.direction = AngularDirection::CW_CLOCKWISE},
                        false);
  pros::delay(200);
  doinkerP = !doinkerP;
  chassis.turnToPoint(-7, -27, 800);
  chassis.moveToPoint(-7, -27, 1500, {.maxSpeed = 50});
  pros::delay(500);
  clamp();
}

void blue2ringLadder() {
  chassis.setPose(50, -23.5, 90);
  chassis.moveToPoint(23.5, -23.5, 1500, {.forwards = false, .maxSpeed = 60},
                      false);
  pros::delay(200);
  clamp();
  moveIntake(1);
  // Ring 1
  chassis.turnToPoint(23.5, -47.5, 800);
  chassis.moveToPoint(23.5, -47.5, 1000);
  chassis.turnToPoint(60, -60, 800, {}, false);
  chassis.moveToPoint(60, -60, 1000);
  pros::delay(200);
  doinkerP = !doinkerP;
  pros::delay(400);
  moveIntake(0);
  chassis.turnToHeading(
      -45, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  pros::delay(200);
  doinkerP = !doinkerP;
  chassis.turnToPoint(4, -27, 800);
  chassis.moveToPoint(4, -27, 1500, {.maxSpeed = 50});
  pros::delay(500);
  clamp();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate(); // Calibrate sensors
  LadyBrownRotation.set_position(0);

  // Start the continuous PID task for ladyBrown lift control
  startContinuousLiftControl();

  // Start the screen task for live updates
  pros::Task screenTask([&]() {
    while (true) {
      // Update piston and brake states
      clampPistonL.set_value(clampP1);
      clampPistonR.set_value(clampP2);
      intakePiston.set_value(intakeP);
      doinkerPiston.set_value(doinkerP);
      doinkerPiston2.set_value(doinkerP2);
      ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      // Set brake modes
      //   leftMotors.set_brake_mode(driveHold ? pros::E_MOTOR_BRAKE_HOLD
      //                                       : pros::E_MOTOR_BRAKE_COAST);
      //   rightMotors.set_brake_mode(driveHold ? pros::E_MOTOR_BRAKE_HOLD
      //                                        : pros::E_MOTOR_BRAKE_COAST);

      // Print robot pose to LCD
      pros::lcd::print(0, "X: %.1f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %.1f", chassis.getPose().y);
      pros::lcd::print(2, "Heading: %.1f", chassis.getPose().theta);
      pros::lcd::print(3, "Lady Brown Angle: %d",
                       LadyBrownRotation.get_position() / 100);
      pros::delay(50);
    }
  });
  pros::Task unstuck([&]() {
    int stallCount = 0;
    while (true) {
      // If a reversal is active, skip unstuck checking.
      if (unstuckOverride) {
        pros::delay(20);
        continue;
      }
      // Debug: Print actual velocity
      double currentVelocity = intakeMotor.get_actual_velocity();
      pros::lcd::print(4, "Intake Vel: %.1f", currentVelocity);

      // Only count consecutive low velocity readings
      if (intakeActive && currentVelocity < 10) {
        stallCount++;
      } else {
        stallCount = 0;
      }

      // If low velocity persists for a few cycles (~60ms), trigger reversal
      if (stallCount > 3) {
        unstuckOverride = true; // Prevent further checks during reversal
        moveIntake(0);          // Reverse intake
        pros::delay(100);
        moveIntake(1);  // Resume intake motion
        stallCount = 0; // Reset counter after reversal
        pros::delay(
            600); // Additional delay before allowing unstuck to run again
        unstuckOverride = false;
      }
      pros::delay(20);
    }
  });
  // In the colorSort task:
  pros::Task colorSort([&]() {
    while (true) {
      // Read the hue and the distance from the color sensor.
      double hue = IntakeColor.get_hue();         // Hue in degrees (0-360)
      int distance = IntakeColor.get_proximity(); // Distance in mm
      IntakeColor.set_led_pwm(200); // Turn on the color sensor LED

      // Only consider the object if it is within ~4 inches (100 mm)
      if (distance <= 100 && intakeActive) {
        if (isRedAlliance) {
          // For red alliance, if a blue ring is detected (hue typically between
          // 200-260°)
          if (hue > 210 && hue < 220) {
            intakeOverride = true;
            moveIntake(0);    // Stop the intake to eject the blue ring
            pros::delay(600); // Hold the intake off for 600ms
            moveIntake(2);    // Reverse for a moment to eject the ring
            pros::delay(300);
            moveIntake(0); // Stop briefly
            pros::delay(100);
            intakeOverride =
                false; // Release override only after the sequence completes
          }
        } else {
          // For blue alliance, if a red ring is detected
          if ((hue >= 0 && hue < 60) || (hue > 340)) {
            intakeOverride = true;
            moveIntake(0);    // Stop the intake
            pros::delay(600); // Wait a moment
            moveIntake(2);    // Reverse for a moment to eject the ring
            pros::delay(300);
            moveIntake(0); // Stop briefly
            pros::delay(100);
            intakeOverride =
                false; // Release override only after the sequence completes
          }
        }
      }
      pros::delay(50); // Check about 20 times per second.
    }
  });
}

/**
 * Runs while the robot is disabled
 */
void disabled() { chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); }

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */
void autonomous() {
  /*
  Case 1: Red Side Negative Corner Auton
  Case 2: Blue Side Negative Corner Auton
  Case 3: Red Side Positive Corner Auton
  Case 4: Blue Side Positive Corner Auton
  Case 5: Skills Autonomous
  Case 6: Blue Solo AWP
  Case 7: Red Solo AWP
  */
  //   driveHold = true;
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Auton Number
  int autonSelected = 3;

  switch (autonSelected) {
  case 1:
    // Blue Six Ring Negative Corner Autonomous
    isRedAlliance = false;
    blueSixRing();
    break;
  case 2:
    // Blue Side Positive Corner Autonomous
    isRedAlliance = false;
    blue2ringLadder();
    break;
  case 3:
    // Red Six Ring Negative Corner Autonomous
    isRedAlliance = true;
    redSolo();
    break;
  case 4:
    // Red Side Positive Corner Autonomous
    isRedAlliance = true;
    red2ringLadder();
    break;
  case 5:
    isRedAlliance = true;
    skills();
    break;
  case 6:
    isRedAlliance = false;
    blue2ring();
    break;
  case 7:
    isRedAlliance = true;
    red2ring();
    break;
  case 8:
  chassis.setPose(59,20,180);
  break;
  }
}

/**
 * Runs in driver control
 */
void opcontrol() {
  driveHold = false;
  ladyBrown(0);
  // controller
  // loop to continuously update motors
  while (true) {
    // get joystick positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.arcade(leftY, rightX);
    // delay to save resources
    pros::delay(10);
    // Within opcontrol() loop:
    if (!intakeOverride) {
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        moveIntake(2);
      } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        moveIntake(1);
      } else {
        // If no driver command is given, only then let auto unstuck control
        // default to off.
        moveIntake(0);
      }
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      clamp();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      if (ladybrown_counter == 0) {
        ladyBrown(1);
        ladybrown_counter++;
      } else if (ladybrown_counter == 1) {
        ladyBrown(3);
        ladybrown_counter++;
      } else {
        ladyBrown(0);
        ladybrown_counter = 0;
      }
    }
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
    //   nextState();
    // }
    // pros::delay(20);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      ladyBrown(0);
      ladybrown_counter = 0;
    }

    pros::delay(20);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      doinkerP = !doinkerP;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      doinkerP2 = !doinkerP2;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      intakeP = !intakeP;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      ladyBrown(1);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      ladyBrown(3);
      pros::delay(500);
      ladyBrown(0);
    }
  }
}
