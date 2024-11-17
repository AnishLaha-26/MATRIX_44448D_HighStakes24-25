#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/error.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
bool IntakeCheck;
bool DoinkerCheck;
bool MogoCheck;
pros::adi::DigitalOut mogo1('A'); 
pros::adi::DigitalOut mogo2('B');

pros::adi::DigitalOut intake('C');

pros::adi::DigitalOut doinker('D');
// motor groups
//pros::MotorGroup leftMotors({5, 4, 3},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-18,-10,-9},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
//pros::MotorGroup rightMotors({-6, -9, -7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::MotorGroup rightMotors({8,17, 7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 3.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.75, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using new 4" omnis
                              400, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
); 

// lateral motion controller
//TO DO: kP of 5 has oscillation, tune kD to remove oscillation
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.0000019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.0000019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    IntakeCheck=false;
    DoinkerCheck = false;
    MogoCheck = false;
    pros::lcd::initialize(); // initialize brain screen
    pros::Motor motor(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
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
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
ASSET(feildTest1_txt);
void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(-113.389, -145.657, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.turnToHeading(90, 100000);
    chassis.moveToPose(-113.389, 152, 0, 100000);
    //chassis.follow(feildTest1_txt, 15, 1000000);
    


}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    chassis.setPose(-62.276, -62.254, 0);
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        pros::Motor motor(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
        motor.move(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightY);
        int Left = (leftMotors.get_actual_velocity(1) + leftMotors.get_actual_velocity(3))/2;
        int Right = (rightMotors.get_actual_velocity(1) + rightMotors.get_actual_velocity(2) + rightMotors.get_actual_velocity(3))/3;
        int driveTemp = rightMotors.get_temperature_all()[0];
        int driveRPM = rightMotors.get_actual_velocity_all()[0];


        controller.print(0, 0, "Dtmp: %d", driveTemp);
        controller.clear_line(0);
        pros::lcd::print(3, "Drive RPM: %d",driveRPM ); // heading
        //std::cout << "Drive : " << driveRPM;

          if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            mogo1.set_value(true);  // Extend mogo pistons
            mogo2.set_value(true);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        
            mogo1.set_value(false);  // Retract mogo pistons
            mogo2.set_value(false);
        }

        // Control intake piston with X button (toggle)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (!IntakeCheck){
                        intake.set_value(true);
                        IntakeCheck=true;
                        pros::delay(500);
                    }
                    else{
                        intake.set_value(false);
                        IntakeCheck=false;
                        pros::delay(500);
                    }
        }

        // Control doinker piston with right arrow button (toggle)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
             if (!DoinkerCheck){
                        doinker.set_value(true);
                        DoinkerCheck=true;
                        pros::delay(500);
                    }
                    else{
                        doinker.set_value(false);
                        DoinkerCheck=false;
                        pros::delay(500);
                    }
        
        }

        // delay to save resources
        pros::delay(2);
    }
}