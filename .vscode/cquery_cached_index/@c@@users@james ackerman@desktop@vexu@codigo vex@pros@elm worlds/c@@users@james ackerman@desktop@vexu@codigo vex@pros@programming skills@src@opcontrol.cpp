#include "main.h"
//#include "okapi/api.hpp"
//#include "definitions.hpp"
//using namespace okapi; //

//TODO: check syntax for ControllerButton for R1/R2/L1/L2/Down/left/right buttons
ControllerButton RightBumperUP(ControllerDigital::R1);
ControllerButton RightBumperDOWN(ControllerDigital::R2);
ControllerButton LeftBumperUP(ControllerDigital::L1);
ControllerButton LeftBumperDOWN(ControllerDigital::L2);
ControllerButton ButtonA(ControllerDigital::A);
ControllerButton ButtonB(ControllerDigital::B);
ControllerButton ButtonX(ControllerDigital::X);
ControllerButton ButtonY(ControllerDigital::Y);
ControllerButton ButtonUP(ControllerDigital::up);
ControllerButton ButtonDOWN(ControllerDigital::down);
ControllerButton ButtonLEFT(ControllerDigital::left);
ControllerButton ButtonRIGHT(ControllerDigital::right);
////////////////////////////////////OPCONTROL///////////////////////////////////////////////////

void opcontrol() {
//
  Controller controller;
  bool STATE = LOW;                        //For Pneumatics //Change to false if this doesn't not work
  int FLYWHEEL_STATE = 1;
  int FLYWHEEL_TARGET = 0;
  int FLYWHEEL_LOW = 400;
  int FLYWHEEL_MID = 500;
  int FLYWHEEL_HIGH = 600;
  linetrackerL.calibrate();
  linetrackerR.calibrate();
  gyro.calibrate();

  while (true)
	{
     // pros::lcd::initialize();
     // pros::lcd::print(0, "Joystick valY: %d", controller.getAnalog(ControllerAnalog::leftY));
     // pros::lcd::print(0, "Joystick valX: %d", controller.getAnalog(ControllerAnalog::leftX));
		 //////////////////////////////CHASSIS(DRIVE)/////////////////////////////////
    // if (abs(controller.getAnalog(ControllerAnalog::leftY)) > 15 ||
     driveController.arcade(controller.getAnalog(ControllerAnalog::leftY), -controller.getAnalog(ControllerAnalog::rightX));

     //////////////////////////////TRANSMISSION/////////////////////////////////
     if (ButtonB.changedToPressed())//
     {
         STATE = !STATE;
         piston.set_value(STATE);
     }
     /////////////////////////////////INTAKE////////////////////////////////////

      liftL.move_voltage(12000*controller.getAnalog(ControllerAnalog::rightY));
      liftR.move_voltage(12000*controller.getAnalog(ControllerAnalog::rightY));
      liftT.move_voltage(12000*controller.getAnalog(ControllerAnalog::rightY));

		 pros::Task::delay(20);
	 }
}
