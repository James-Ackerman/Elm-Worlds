#pragma once

#include "main.h"
//#include "okapi/api.hpp"
//using namespace okapi;



 //Sensor definitions
 const int DRIVE_PNEUMATIC = 3;     //port A //change to A if this doesn't work
 const int LINE_TRACKER_LEFT = 2;   // use "#define SENSOR_NAME_HERE portnumber" if const int doesnt work
 const int LINE_TRACKER_RIGHT = 1;
 const int GYRO_PORT = 4;
 const int ULTRASONIC_OUT = 5;
 const int ULTRASONIC_IN = 6;

 // Chassis definition
 // Assume this is correct for now
 // TODO: Update with the real robot values
 const int DRIVE_MOTOR_LEFT_1 = 16;   //1,2,3 =Top, Bottom, Front
 const int DRIVE_MOTOR_LEFT_2 = -15;
 const int DRIVE_MOTOR_LEFT_3 = 14;
 const int DRIVE_MOTOR_RIGHT_1 = -11;   //1,2,3 = Top, Bottom, Front
 const int DRIVE_MOTOR_RIGHT_2 = 12;
 const int DRIVE_MOTOR_RIGHT_3 = -13;
 const auto WHEEL_DIAMETER = 4_in; //4_in
 const auto CHASSIS_WIDTH = 12.75_in; //13_in

 const int LIFT_MOTOR_RIGHT = 19;
 const int LIFT_MOTOR_LEFT = -18;
 const int LIFT_MOTOR_TOP = -10;
 const int ROTATOR_MOTOR = -17;    // 1 for side roller




 // Controller object creation
 //TODO: Reverse motors that need to be reversed

inline AbstractMotor::GearsetRatioPair torqueTrans = AbstractMotor::gearset::green*(5.0/3.0);
inline AbstractMotor::GearsetRatioPair speedTrans = AbstractMotor::gearset::green*(3.0/5.0);
inline auto driveController = ChassisControllerFactory::create(
   {DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3},
   {DRIVE_MOTOR_LEFT_1, DRIVE_MOTOR_LEFT_2, DRIVE_MOTOR_LEFT_3},
   // IterativePosPIDController::Gains{0.5, 0, 0},
   // IterativePosPIDController::Gains{0.01, 0.03, 0},
   // IterativePosPIDController::Gains{0.05, 0, 0},
   torqueTrans,
   {WHEEL_DIAMETER, CHASSIS_WIDTH}
 );

inline auto descorerController = AsyncControllerFactory::posIntegrated({LIFT_MOTOR_LEFT,LIFT_MOTOR_RIGHT,LIFT_MOTOR_TOP});

inline MotorGroup base({DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3,
DRIVE_MOTOR_LEFT_1, DRIVE_MOTOR_LEFT_2, DRIVE_MOTOR_LEFT_3});

inline Motor liftL(LIFT_MOTOR_LEFT);
inline Motor liftR(LIFT_MOTOR_RIGHT);
inline Motor flywheel(ROTATOR_MOTOR);
inline Motor liftT(LIFT_MOTOR_TOP);
 inline pros::ADIDigitalOut piston(DRIVE_PNEUMATIC);         //Piston on DRIVE_PNEUMATIC port
 inline pros::ADILineSensor linetrackerL(LINE_TRACKER_LEFT); //Line tracker on LINE_TRACKER_LEFT port
 inline pros::ADILineSensor linetrackerR(LINE_TRACKER_RIGHT); //Line tracker on LINE_TRACKER_LEFT port
 inline pros::ADIAnalogIn gyro (GYRO_PORT);
 inline pros::ADIUltrasonic ultrasonic1(ULTRASONIC_IN, ULTRASONIC_OUT);


//#endif /* end of include guard:  */
