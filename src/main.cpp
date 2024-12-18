#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "lemlib/pose.hpp"


// controller init
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// boolean variables to toggle pnematics
bool IntakeCheck;
bool DoinkerCheck;
bool MogoCheck;

// pins of pneumatics
pros::adi::DigitalOut mogoMech('G');
pros::adi::DigitalOut intake('H');
pros::adi::DigitalOut doinker('F');

// motor groups
pros::MotorGroup leftMotors({-18, -5, -9},
                            pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({4, 15, 6},
                             pros::MotorGearset::blue); // right motor group

//Intake Motors 
pros::Motor intake1(16,pros::MotorGearset::blue);
pros::Motor intake2(-20,pros::MotorGearset::blue);

// port of IMU
pros::Imu imu(11);

// tracking wheels
// PORTS OF TRACKING WHEELS
//  horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(3);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(1);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot
// (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275,
                                 3.25); // TO DO: Offest of tracking wheels
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot
// (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.5);


// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    12.75,                      // Change track width to robot
    lemlib::Omniwheel::OLD_325, // using new 4" omnis
    450,                        // drivetrain rpm is 450
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// TO DO: Tune PID controls again following tutorial:
// https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html lateral
// motion controller
// TO DO: kP of 5 has oscillation, tune kD to remove oscillation
lemlib::ControllerSettings
    lateral_controller(12, // proportional gain (kP)
                       0,  // integral gain (kI)
                       39,  // derivative gain (kD) //TO DO: increase to eliminate oscillations
                       3,  // anti windup
                       1,  // small error range, in inches
                       100,  // small error range timeout, in milliseconds
                       3,  // large error range, in inches
                       500,  // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// Angular controller tuned
lemlib::ControllerSettings
    angular_controller(4,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       39, // derivative gain (kD)
                       0,  // anti windup
                        1,  // Small error range in degrees
                      100, // Small error timeout
                        5,   // Large error range in degrees
                       500, // Large error timeout in milliseconds
                        0    // Maximum acceleration (not critical here)
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
    throttleCurve(3,  // joystick deadband out of 127
                  10, // minimum output where drivetrain will move out of 127
                  1.0000019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,        // joystick deadband out of 127
               10,       // minimum output where drivetrain will move out of 127
               1.000019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  IntakeCheck = false;
  DoinkerCheck = false;
  MogoCheck = false;
  doinker.set_value(true);
  DoinkerCheck = true;
  pros::lcd::initialize(); // initialize brain screen
  pros::Motor motor(1, pros::v5::MotorGears::blue,
                    pros::v5::MotorUnits::degrees);
  chassis.calibrate(); // calibrate sensors


  // the default rate is 50. however, if you need to change the rate, you
  // can do the following.
  // lemlib::bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of
  // 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging
  pros::Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */


void mogoToggle() {
  if (!MogoCheck) {
    mogoMech.set_value(true);
    MogoCheck = true;
    pros::delay(500);
  } else {
    mogoMech.set_value(false);
    MogoCheck = false;
    pros::delay(500);
  }
}

void intakeToggle(){
if (!IntakeCheck) {
        intake.set_value(true);
        IntakeCheck = true;
        pros::delay(500);
      } else {
        intake.set_value(false);
        IntakeCheck = false;
        pros::delay(500);
      }
}
void doinkerToggle(){
    if (!DoinkerCheck) {
            doinker.set_value(true);
            DoinkerCheck = true;
            pros::delay(500);
        } else {
            doinker.set_value(false);
            DoinkerCheck = false;
            pros::delay(500);
        }
}


void autonomous() {
  //       //put this after op control
  //   doinkerToggle();
  chassis.setPose(0, 0, 0); //this is intial set position (robot start zone)
    chassis.moveToPoint(0,-17,1000,{.forwards = false}, false); //initial move forward to clear rings
    chassis.moveToPoint(0,-10,1000,{.forwards = true}, false); // //move back to allow robot to have space to turn (for wall stake)
    chassis.moveToPose(
        18,                 // x-coordinate
        -11,                 // y-coordinate
        -93,                // theta (heading in degrees)
        1600,               // timeout in milliseconds
        {.forwards = false, .horizontalDrift = 8, .lead = 0.6,}      
    ); // boomerang algorithm to perform curved turn to wall stake scoring location.
    
    chassis.moveToPoint(18.5, -8 ,1000, {.forwards = false}, false); //move forward to score wall stake

    intake1.move_relative(-375, 600); // Move intake1 360 degrees
    intake2.move_relative(-375, 600); // Move intake2 360 degrees (places ring onto wall stake)
     // Wait for the motors to complete their movement
    while (fabs(intake1.get_position() - intake1.get_target_position()) > 5 || 
           fabs(intake2.get_position() - intake2.get_target_position()) > 5) {
        pros::delay(10); // Check every 10ms
   }
  

  // chassis.moveToPoint(13, 0,1000, {.forwards = false}, false); //move back from the wall to allow turning
  // chassis.turnToHeading(-134,500); //turn to collect the other rings
  // doinkerToggle();
  // intake1.move_velocity(-600); //turn on intake to start collecting rings
  // intake2.move_velocity(-600);
  // chassis.moveToPoint(-13, -46,1750, {.forwards = true}, false); //move to this set point (ensuring rings are collected)
  // intake1.move_velocity(0);
  // intake2.move_velocity(0);

  // chassis.moveToPoint(-8, -33,1000, {.forwards = false}, false);
  // // chassis.turnToHeading(-241,1000);
  // // mogoToggle();
  // // chassis.moveToPoint(-21, -37,1000, {.forwards = false}, false);
  // // mogoToggle();
  // // intake1.move_velocity(-600); //turn on intake to start collecting rings
  // // intake2.move_velocity(-600);

}






void opcontrol() {

  // chassis.moveToPoint(-21, -37,1000, {.forwards = false}, false);
  // mogoToggle();
  // intake1.move_velocity(-600); //turn on intake to start collecting rings
  // intake2.move_velocity(-600);
  // //       //put this after op control
  //   //doinkerToggle();
  //   chassis.setPose(0, 0, 0); //this is intial set position (robot start zone)
  //   chassis.moveToPoint(0,-17,1000,{.forwards = false}, false); //initial move forward to clear rings
  //   chassis.moveToPoint(0,-10,1000,{.forwards = true}, false); // //move back to allow robot to have space to turn (for wall stake)
  //   chassis.moveToPose(
  //       18,                 // x-coordinate
  //       -11,                 // y-coordinate
  //       -93,                // theta (heading in degrees)
  //       1600,               // timeout in milliseconds
  //       {.forwards = false, .horizontalDrift = 8, .lead = 0.6,}      
  //   ); // boomerang algorithm to perform curved turn to wall stake scoring location.
    
  //   chassis.moveToPoint(18.5, -8 ,1000, {.forwards = false}, false); //move forward to score wall stake

  //   intake1.move_relative(-375, 600); // Move intake1 360 degrees
  //   intake2.move_relative(-375, 600); // Move intake2 360 degrees (places ring onto wall stake)
  //      while (fabs(intake1.get_position() - intake1.get_target_position()) > 5 || 
  //          fabs(intake2.get_position() - intake2.get_target_position()) > 5) {
  //       pros::delay(10); // Check every 10ms
  //  }

  // chassis.moveToPoint(13, 0,1000, {.forwards = false}, false); //move back from the wall to allow turning
  // chassis.turnToHeading(-134,500); //turn to collect the other rings
  // doinkerToggle();
  // intake1.move_velocity(-600); //turn on intake to start collecting rings
  // intake2.move_velocity(-600);
  // chassis.moveToPoint(-13, -46,1500, {.forwards = true}, false); //move to this set point (ensuring rings are collected)
  // intake1.move_velocity(0);
  // intake2.move_velocity(0);

  // chassis.moveToPoint(-10, -40,1700, {.forwards = false}, false);
  // chassis.turnToHeading(-241,1700);
  // mogoToggle();
  // // chassis.moveToPoint(-20, -36,1000, {.forwards = false}, false);
  // // mogoToggle();
  // // intake1.move_velocity(-600); //turn on intake to start collecting rings
  // // intake2.move_velocity(-600);

  //   //ABOVE HERE



  //controller
  // loop to continuously update motors
  //chassis.setPose(0, 0, 0);
  while (true) {

    //DRIVE CHASSSIS CODE
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    pros::Motor motor(20, pros::v5::MotorGears::blue,
                      pros::v5::MotorUnits::degrees);
    motor.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    // move the chassis with curvature drive
    chassis.arcade(leftY, rightY);

 
  //INTAKE CODE
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        intake1.move_voltage(-12000);
        intake2.move_voltage(-12000);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake1.move_voltage(12000);
        intake2.move_voltage(12000);
    }else{
      intake1.move_voltage(0);
      intake2.move_voltage(0);
    }

    
  


    //MOGO MECH CONTROL CODE
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      mogoToggle();
    }

    //INTAKE UP/DOWN CONTROL CODE
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      intakeToggle();
    }

    //DOINKER CONTROL CODE
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      doinkerToggle();
    }


    //CONTROLLER SCREEN OUTPUTS
    int Left = (leftMotors.get_actual_velocity(1) +
                leftMotors.get_actual_velocity(3)) /
               2;
    int Right = (rightMotors.get_actual_velocity(1) +
                 rightMotors.get_actual_velocity(2) +
                 rightMotors.get_actual_velocity(3)) /
                3;
    int driveTemp = rightMotors.get_temperature_all()[0];
    int driveRPM = rightMotors.get_actual_velocity_all()[0];

    controller.print(0, 0, "Dtmp: %d", driveTemp);
    controller.clear_line(0);
    pros::lcd::print(3, "Drive RPM: %d", driveRPM); // heading
    // std::cout << "Drive : " << driveRPM;





    // delay to save resources
    pros::delay(2);
  }
}