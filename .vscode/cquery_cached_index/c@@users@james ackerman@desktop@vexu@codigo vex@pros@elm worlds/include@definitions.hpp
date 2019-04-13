#pragma once
#include "main.h"

//Sensor ports
const int PROXIMITY_TOP = 1;
const int PROXIMITY_BOTTOM = 2;
const int GYRO_1 = 3;
const int GYRO_2 = 4;
const int SHIFTER_PNEUMATIC = 5;
const int DRIVE_PNEUMATIC = 6;
const int LINE_TRACKER_RIGHT = 7;
const int LINE_TRACKER_LEFT = 8;

//Sensor definitions
inline pros::ADIDigitalOut pistonT(DRIVE_PNEUMATIC);         //Transmission solenoid
inline pros::ADIDigitalOut pistonS(SHIFTER_PNEUMATIC);       //Shifter solenoid
inline pros::ADIDigitalIn  ProxTOP(PROXIMITY_TOP);           //Top proximity sensor
inline pros::ADIDigitalIn  ProxBOT(PROXIMITY_BOTTOM);        //Bottom proximity sensor
inline pros::ADILineSensor linetrackerL(LINE_TRACKER_LEFT);  //Left line tracker
inline pros::ADILineSensor linetrackerR(LINE_TRACKER_RIGHT); //Right line tracker
inline ADIGyro gyro1(GYRO_1, 0.1);                           //Top gyro (C)
inline ADIGyro gyro2(GYRO_2, 0.1);                           //Bottom gyro (D)

//Chassis motor ports/dimensions. Update with the real robot values
const int DRIVE_MOTOR_RIGHT_1 = -1;   //1,2,3 = Top, Bottom, Front
const int DRIVE_MOTOR_RIGHT_2 = 2;
const int DRIVE_MOTOR_RIGHT_3 = -3;
const int DRIVE_MOTOR_LEFT_1 = 4;     //1,2,3 = Top, Bottom, Front
const int DRIVE_MOTOR_LEFT_2 = -5;
const int DRIVE_MOTOR_LEFT_3 = 6;
const auto WHEEL_DIAMETER = 4_in;     //4_in
const auto CHASSIS_WIDTH = 12.25_in;  //13_in

// Motor port definitions. Update with the real robot values
const int FLYWHEEL_MOTOR = 16;
const int INTAKE_MOTOR = 17;
const int ROLLER_MOTOR = 18;
const int DESCORER_MOTOR = -19;
const int INDEXER_MOTOR = 20;

//Filter declarationS
inline EKFFilter kalmanfilterFW (0.2, 0.05);
inline DemaFilter demaFilter (0.2, 0.05);

//MotorGroups
inline MotorGroup base({DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3, DRIVE_MOTOR_LEFT_1, DRIVE_MOTOR_LEFT_2, DRIVE_MOTOR_LEFT_3});
inline MotorGroup baseL({DRIVE_MOTOR_LEFT_1, DRIVE_MOTOR_LEFT_2, DRIVE_MOTOR_LEFT_3});
inline MotorGroup baseR({DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3});

// Controller object creation
inline AbstractMotor::GearsetRatioPair torqueTrans = AbstractMotor::gearset::green*(5.0/3.0);
inline AbstractMotor::GearsetRatioPair speedTrans = AbstractMotor::gearset::green*(3.0/5.0);
inline auto driveController = ChassisControllerFactory::create(
   {DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3},
   {DRIVE_MOTOR_LEFT_1, DRIVE_MOTOR_LEFT_2, DRIVE_MOTOR_LEFT_3},
   IterativePosPIDController::Gains{0.000001, 0, 0},
   IterativePosPIDController::Gains{0, 0, 0},
   IterativePosPIDController::Gains{0, 0, 0},
   speedTrans,
   {WHEEL_DIAMETER, CHASSIS_WIDTH}
  );

//Individual motor definitions (for easy voltage control)
inline Motor driveR(DRIVE_MOTOR_RIGHT_2);
inline Motor driveL(DRIVE_MOTOR_LEFT_2);
inline Motor driver(DRIVE_MOTOR_RIGHT_2);
inline Motor flywheel(FLYWHEEL_MOTOR, true, AbstractMotor::gearset::blue);	  		//motor on FLYWHEEL_MOTOR port
inline Motor intake(INTAKE_MOTOR);             //motor on INTAKE_MOTOR port
inline Motor roller(ROLLER_MOTOR);
inline Motor descorer(DESCORER_MOTOR);
inline Motor indexer(INDEXER_MOTOR);           //motor on INDEXER_MOTOR port
//TODO: if velIntegrated doesn't work, use posIntegrated
//TODO: use {DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3} etc if baseR/L doesnt work
inline auto driveControllerR = AsyncControllerFactory::velIntegrated(baseR);                     //Controls right side of chassis. To be used for braking that specific side
inline auto driveControllerL = AsyncControllerFactory::velIntegrated(baseL);                     //Controls left side of chassis. To be used for braking that specific side
inline auto descorerController = AsyncControllerFactory::posIntegrated(DESCORER_MOTOR);
//Flywheel Global Variables. Each explained in definitions.cpp
const int FW_LOOP_SPEED = 20;
const int FW_MAX_POWER  = 12000;
extern float           motor_velocity;
extern long            target_velocity;
extern float           current_error;
extern float           last_error;
extern float           gain;
extern float           drive;
extern float           drive_at_zero;
extern long            first_cross;
extern float           drive_approx;
extern long            motor_drive;

extern int  lineR;
extern bool passedLineR;
extern int  current_speedR;
extern int thresholdR;
extern bool endTaskR;

extern bool pgfLeft;
extern bool pgbLeft;
extern int  lineL;
extern int  current_speedL;
extern int thresholdL;
extern bool endTaskL;

//Function declarations. Definitions appear in definitions.cpp
void FwMotorSet( int value );
void FwVelocitySet( int vel, float predicted_drive );
void sgn(float x);
void FwControlUpdateVelocityTbh();
void FwControlTask(void* param);
void Noslackmove(int distance);
void Noslackturn(int, int, int);
void PIDGyroTurn( float, QTime, float, float, float, float);
void lineFW_NEW(float, float, int);
void lineFW_OLD(float, float, int);
void alignWithLine(int, int, int);
// void RightCorrect(void*param);
// void LeftCorrect(void*param);
//
inline ControllerButton RightBumperUP(ControllerDigital::R1);
inline ControllerButton RightBumperDOWN(ControllerDigital::R2);
inline ControllerButton LeftBumperUP(ControllerDigital::L1);
inline ControllerButton LeftBumperDOWN(ControllerDigital::L2);
inline ControllerButton ButtonA(ControllerDigital::A);
inline ControllerButton ButtonB(ControllerDigital::B);
inline ControllerButton ButtonX(ControllerDigital::X);
inline ControllerButton ButtonY(ControllerDigital::Y);
inline ControllerButton ButtonUP(ControllerDigital::up);
inline ControllerButton ButtonDOWN(ControllerDigital::down);
inline ControllerButton ButtonLEFT(ControllerDigital::left);
inline ControllerButton ButtonRIGHT(ControllerDigital::right);
