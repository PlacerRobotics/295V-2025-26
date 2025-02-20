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
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
    linearController(8,   // proportional gain (kP)
                     0,   // integral gain (kI)
                     4,   // derivative gain (kD)
                     0.5, // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(1.5, // proportional gain (kP)
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
// Fourth Stake Path
ASSET(fourthStake_txt);

bool clampP1 = false;
bool clampP2 = false;
bool intakeP = false;
bool doinkerP = false;

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

void moveIntake(int state) {
  // Full voltage commands for off, positive, and negative directions
  const int voltages[3] = {0, 120 * 127, -120 * 127};
  if (state < 0 || state > 2) {
    return;
  }
  // Command the motor with the desired voltage.
  intakeMotor.move_voltage(voltages[state]);

  // If moving in the positive direction, check if the motor is stuck.
  if (state == 1) {
    // Allow a brief moment for the voltage command to take effect.
    pros::delay(200);
    // Get the actual velocity (in rpm); adjust the threshold as needed.
    double velocity = intakeMotor.get_actual_velocity();
    if (fabs(velocity) < 10) { // If the motor isn’t spinning as expected...
      // Reverse for 200ms using the negative voltage command.
      intakeMotor.move_voltage(voltages[2]);
      pros::delay(200);
      // Resume the positive command.
      intakeMotor.move_voltage(voltages[1]);
    }
  }
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

// Clamp Pistons
pros::adi::DigitalOut clampPistonL(clampPistonl_p);
pros::adi::DigitalOut clampPistonR(clampPistonr_p);

// Intake Pistons
pros::adi::DigitalOut intakePiston(intake_piston_p);

// Doinker Pistons
pros::adi::DigitalOut doinkerPiston(doinker_piston_p);
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
  const double kp = 0.5; // Lower proportional gain
  const double tolerance =
      5.0; // Allowable error (degrees) before we're "close enough"
  const double deadband =
      10.0; // If error is smaller than 10°, command no motion
  const int minVelocity = 10; // Minimum velocity (rpm) when commanding movement
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
  const float positions[4] = {1, 33, 50, 150};
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
  chassis.setPose(-58, 0, 270);

  // Movement 1
  // Moves forward to align with the stake
  // moveIntake(1);
  // pros::delay(300);
  // chassis.moveToPose(-47.5, 0, -45, 600, {.maxSpeed = 127}, false);
  // moveIntake(0);
  // // Moves to the stake backwards
  // chassis.turnToHeading(0, 500);
  ladyBrown(3);
  pros::delay(500);
  ladyBrown(0);
  chassis.turnToPoint(-48.5, -27, 600, {.forwards = false});
  chassis.moveToPoint(-47.5, -27, 800, {.forwards = false}, false);
  // clamps the stake
  pros::delay(200);
  clamp();

  // Movement 2
  // Moves to first ring
  chassis.turnToPoint(-24, -24, 600, {});
  moveIntake(0);
  pros::delay(500);
  moveIntake(1);
  chassis.moveToPoint(-24, -24, 800);
  pros::delay(500);

  // Movement 3
  // Moves to the center
  chassis.turnToPoint(0, -41, 600);
  chassis.moveToPoint(0, -41, 800, {}, false);
  // Wall stake up
  ladyBrown(1);
  // Moves to second ring
  chassis.turnToPoint(24, -48, 800);
  chassis.moveToPoint(24, -48, 1000);

  // Movement 4
  // Moves to middle
  chassis.moveToPose(0, -44, 90, 2000, {.forwards = false}, false);
  moveIntake(0);
  // Moves to third ring
  chassis.turnToHeading(180, 1000);
  ladyBrown(2);
  pros::delay(500);
  moveIntake(1);
  chassis.moveToPoint(0, -61, 1200, {}, false);
  pros::delay(500);
  ladyBrown(3);
  pros::delay(500);

  // Movement 5
  // Moves to middle
  chassis.moveToPoint(0, -50, 600, {.forwards = false}, false);
  ladyBrown(0);
  // Moves to fourth ring
  chassis.turnToPoint(-23.5, -50, 800);
  chassis.moveToPoint(-23.5, -50, 800, {}, false);

  // Movement 6 & 7
  chassis.moveToPoint(-57, -47.5, 2100, {.maxSpeed = 30}, false);
  pros::delay(500);

  // Movement 8
  // chassis.turnToPoint(-47.5, -59, 800);
  chassis.moveToPose(-47.5, -59, 90, 1200, {}, false);

  // Movement 9
  chassis.turnToPoint(-60, -60, 500, {.forwards = false});
  chassis.moveToPoint(-60, -60, 800, {.forwards = false}, false);
  moveIntake(0);
  clamp();

  /*
  Second Clamp Movements
  */

  // Movement 10
  chassis.turnToPoint(-47.5, 26, 1000, {.forwards = false});
  chassis.moveToPose(-47.5, 26, 180, 2000, {.forwards = false}, false);
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
  chassis.turnToPoint(0, 41, 600);
  chassis.moveToPoint(0, 41, 800, {}, false);
  // Wall stake up
  ladyBrown(1);
  // Moves to second ring
  chassis.turnToPoint(24, 48, 800);
  chassis.moveToPoint(24, 48, 1000);

  // Movement 13
  // Moves to middle
  chassis.moveToPose(0, 44, 90, 2000, {.forwards = false}, false);
  moveIntake(0);
  // Moves to third ring
  chassis.turnToHeading(0, 1000);
  ladyBrown(2);
  pros::delay(500);
  moveIntake(1);
  chassis.moveToPoint(0, 61, 1200, {}, false);
  ladyBrown(3);
  pros::delay(500);

  // Movement 14
  // Moves to middle
  chassis.moveToPoint(0, 50, 1000, {.forwards = false}, false);
  ladyBrown(0);
  // Moves to fourth ring
  chassis.turnToPoint(-23.5, 50, 800);
  chassis.moveToPoint(-23.5, 50, 800, {}, false);

  // Movement 15 & 16
  chassis.moveToPoint(-57, 47.5, 2000, {.maxSpeed = 30}, false);
  pros::delay(500);

  // Movement 17
  chassis.turnToPoint(-47.5, 59, 800);
  chassis.moveToPoint(-47.5, 59, 1200, {}, false);

  // Movement 18
  chassis.turnToPoint(-60, 60, 500, {.forwards = false});
  chassis.moveToPoint(-60, 60, 800, {.forwards = false}, false);
  moveIntake(0);
  clamp();

  /*

  // Third Stake movements

  // */
  // Moves forward a bit
  chassis.moveToPoint(-54, 54, 500);

  // Movement 19
  // chassis.turnToPoint(23.5, 23.5, 800);
  // chassis.moveToPoint(23.5, 23.5, 1500);
  chassis.turnToPoint(-12, 51, 600);
  chassis.follow(thirdStake_txt, 14, 1500);
  pros::delay(1200);
  moveIntake(1);
  pros::delay(500);
  moveIntake(0);

  // Movement 20
  chassis.turnToPoint(52, -3, 800, {.forwards = false});
  chassis.moveToPoint(52, -3, 800, {.forwards = false}, false);
  pros::delay(200);
  clamp();
  pros::delay(300);
  moveIntake(1);

  // Movement 21
  chassis.turnToPoint(23.5, -23.5, 800);
  chassis.moveToPoint(23.5, -23.5, 1200, {}, false);
  pros::delay(500);
  moveIntake(0);

  // Movement 22
  chassis.turnToPoint(0, 0, 800);
  chassis.moveToPoint(0, 0, 1200, {});
  pros::delay(1200);
  moveIntake(1);
  pros::delay(1000);

  // Movement 23
  chassis.turnToPoint(41.5, 41.5, 800, {}, false);
  moveIntake(0);
  chassis.moveToPoint(41.5, 41.5, 1500);
  pros::delay(1000);
  moveIntake(1);

  // Movement 24
  chassis.turnToPoint(39, 39, 800, {.forwards = false});
  chassis.moveToPoint(39, 39, 800, {.forwards = false});

  chassis.turnToPoint(59, 47.5, 600);
  chassis.moveToPoint(59, 47.5, 800, {}, false);

  // Movement 25
  chassis.turnToPoint(39, 39, 600, {.forwards = false});
  chassis.moveToPoint(39, 39, 800, {.forwards = false});

  chassis.turnToPoint(47.5, 59, 600);
  chassis.moveToPoint(47.5, 59, 800);

  // Movement 26
  moveIntake(0);
  chassis.turnToPoint(61, 65, 800);
  doinkerP = !doinkerP;
  chassis.moveToPose(61, 68, 90, 1000);
  pros::delay(500);
  chassis.turnToHeading(-135, 1000,
                        {.direction = AngularDirection::CW_CLOCKWISE});
  // pros::delay(1000);
  chassis.moveToPoint(60, 60, 500, {.forwards = false}, false);
  clamp();
  pros::delay(50);
  doinkerP = !doinkerP;

  /*
  Fourth Stake Movements
  */

  // Movement 27
  chassis.moveToPoint(54, 54, 500);
  chassis.turnToPoint(27.5, 20, 1000, {.forwards = false});
  chassis.follow(fourthStake_txt, 14, 2000, false, false);

  // Movement 28
  // chassis.turnToPoint(59, -23.5, 600, {.forwards = false});
  // chassis.moveToPoint(59, -23.5, 1000, {.forwards = false}, false);

  // Movement 29
  chassis.moveToPoint(62.5, -62.5, 1500, {.forwards = false}, false);
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
  // chassis.turnToPoint(43.5, 30, 500, {.forwards = false});
  // chassis.follow(blueNegative1_txt, 14, 1500, false, false);
  chassis.moveToPoint(24, 23.5, 800, {.forwards = false}, false);
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
  // Ring 4 Corner
  chassis.turnToPoint(59, 59, 800);
  chassis.moveToPoint(59, 59, 1200);
  // Ring 5 Corner
  chassis.moveToPoint(52.5, 52.5, 800, {.forwards = false}, false);
  intakeP = !intakeP;
  chassis.moveToPoint(59, 59, 1000, {}, false);
  pros::delay(200);
  intakeP = !intakeP;
  // Back up
  chassis.moveToPoint(52.5, 52.5, 800, {.forwards = false});
  // Ring 6 Middle
  chassis.turnToPoint(47.5, 10, 800, {}, false);
  intakeP = !intakeP;
  chassis.moveToPoint(47.5, 10, 1200, {}, false);
  intakeP = !intakeP;
  pros::delay(500);
}

void blueGoalRush() {
  // Blue Positive Side Rush
  chassis.setPose(49, -47.5, 270);
  // Rush to the stake
  chassis.moveToPoint(10, -43, 1000, {});
  moveIntake(1);
  pros::delay(500);
  moveIntake(0);
  doinkerP = !doinkerP;
  chassis.moveToPoint(45, -47.5, 1000, {.forwards = false}, false);
  doinkerP = !doinkerP;
  chassis.turnToPoint(23.5, -23.5, 800, {.forwards = false});
  chassis.moveToPose(23.5, -23.5, -10, 2000, {.forwards = false}, false);
  pros::delay(500);
  clamp();
  moveIntake(1);
  chassis.turnToPoint(55, -55, 1000);
  chassis.moveToPoint(55, -55, 1200);
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
      ladyBrownMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      // Set brake modes
      leftMotors.set_brake_mode(driveHold ? pros::E_MOTOR_BRAKE_HOLD
                                          : pros::E_MOTOR_BRAKE_COAST);
      rightMotors.set_brake_mode(driveHold ? pros::E_MOTOR_BRAKE_HOLD
                                           : pros::E_MOTOR_BRAKE_COAST);

      // Print robot pose to LCD
      pros::lcd::print(0, "X: %.1f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %.1f", chassis.getPose().y);
      pros::lcd::print(2, "Heading: %.1f", chassis.getPose().theta);
      pros::lcd::print(3, "Lady Brown Angle: %d",
                       LadyBrownRotation.get_position() / 100);
      pros::delay(50);
    }
  });
  // pros::Task liftControlTask([] {
  //   while (true) {
  //     liftControl();
  //     pros::delay(10);
  //   }
  // });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

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
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Auton Number
  int autonSelected = 5;

  switch (autonSelected) {
  case 1:
    blueSixRing();
    break;
  case 2:
    blueGoalRush();
    break;
  case 3:
    // Red Side Negative Corner Autonomous
    break;
  case 4:
    // Red Side Positive Corner Autonomous
    break;
  case 5:
    skills();
    break;
  case 6:
    // Blue Solo AWP
    blueSolo();
    break;
  }
}

/**
 * Runs in driver control
 */
void opcontrol() {
  driveHold = false;
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
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      moveIntakeOP(1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      moveIntakeOP(2);
    } else {
      moveIntakeOP(0);
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

// Movement 3
// Moves to the almost center
// chassis.turnToPoint(-2, -43, 800);
// chassis.moveToPoint(-2, -43, 800, {}, false);
// // Wall stake up
// ladyBrown(1);
// // Moves to second ring
// chassis.turnToPoint(24, -49, 800);
// chassis.moveToPoint(24, -49, 1000);

// // Movement 4
// // Moves to third ring
// chassis.turnToPoint(9.5, -55, 1200, {}, false);
// moveIntake(0);
// pros::delay(200);
// ladyBrown(2);
// pros::delay(500);
// moveIntake(1);
// chassis.moveToPoint(9.5, -55, 1200, {}, false);
// pros::delay(500);
// chassis.turnToHeading(180, 800, {}, false);
// chassis.moveToPoint(8, -59, 500, {}, false);
// ladyBrown(3);
// pros::delay(1000);
// Moves to the almost center