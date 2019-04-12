#include "main.h"
//maybe move to definitions.hpp

//
////////////////////////////////////OPCONTROL////////////////////////////////////////////////////////////////////////////////////////////////////////
void opcontrol()
{
  //Initial opcontrol settings
  Controller controller;
  bool STATE_TRANSMISSION = LOW;
  bool STATE_SHIFTER = LOW;
  bool SHIFTER_TOGGLE = 0;
  int FLYWHEEL_LOW = 400;
  int FLYWHEEL_MID = 520;
  int FLYWHEEL_HIGH = 600;
  pros::Task FwControl(FwControlTask);
  flywheel.setGearing(AbstractMotor::gearset::blue);
  pros::lcd::set_text(7, "OPCONTROL");
  while (true)
	{
     ////////////////////////////////////LCD////////////////////////////////////
     //pros::lcd::print(2, "GYRO 1: %f\n", gyro1.get());
     //pros::lcd::print(3, "GYRO 2: %f\n", -1*gyro2.get());
     // pros::lcd::print(2, "LINE_TRACKER_RIGHT: %.d\n", linetrackerL.get_value());
     // pros::lcd::print(3, "LINE_TRACKER_LEFT: %.d\n", linetrackerL.get_value());
     pros::lcd::print(2, "PROXIMITY_TOP: %d\n", ProxTOP.get_value());
     pros::lcd::print(2, "SHIFTER_TOGGLE: %d\n", SHIFTER_TOGGLE);
     pros::lcd::print(4, "GYRO COMBINED: %f\n", (((-1*gyro2.get())+gyro1.get())/2));
     pros::lcd::print(6, "Battery Level: %f\n", pros::battery::get_capacity());

		 ////////////////////////////////CHASSIS(DRIVE)/////////////////////////////
     driveController.arcade(controller.getAnalog(ControllerAnalog::leftY), -controller.getAnalog(ControllerAnalog::rightX));

     ////////////////////////////////TRANSMISSION///////////////////////////////
     if (ButtonB.changedToPressed())//
     {
         STATE_TRANSMISSION = !STATE_TRANSMISSION;
         pistonT.set_value(STATE_TRANSMISSION);
     }

     if (ButtonY.changedToPressed())
     {
         STATE_SHIFTER = !STATE_SHIFTER;
         pistonS.set_value(STATE_SHIFTER);
     }
     //////////////////////////////////SHIFTER//////////////////////////////////
     if (ButtonA.changedToPressed())
     {
         SHIFTER_TOGGLE =! SHIFTER_TOGGLE;
     }

     if (ButtonDOWN.changedToPressed())          //Flywheel MID conditions
     {
       //gain = 0.1;
       SHIFTER_TOGGLE = 1;
       while((ProxTOP.get_value() == 0) && SHIFTER_TOGGLE == 1)
       {
         indexer.move_voltage(-12000);
         intake.move_voltage(12000);     //maybe negative
         if (ButtonA.changedToPressed())
         {
           SHIFTER_TOGGLE = 0;
         }
         else;
       }
       while((ProxTOP.get_value() == 1)  && SHIFTER_TOGGLE == 1)
       {
         if (ButtonA.changedToPressed())
         {
           SHIFTER_TOGGLE = 0;
         }
         else;
       }

       pistonS.set_value(HIGH);
       pros::delay(150);

       while((ProxTOP.get_value() == 0)  && SHIFTER_TOGGLE == 1)
       {
         if (ButtonA.changedToPressed())
         {
           SHIFTER_TOGGLE = 0;
         }
         else;
       }
       while((ProxTOP.get_value() == 1) && SHIFTER_TOGGLE == 1)
       {
         if (ButtonA.changedToPressed())
         {
           SHIFTER_TOGGLE = 0;
         }
         else;
       }
       pistonS.set_value(LOW);

       indexer.move_voltage(0);
       intake.move_voltage(0);
     }

     //////////////////////////////////INTAKE///////////////////////////////////
     if (RightBumperDOWN.isPressed() && RightBumperUP.isPressed())
     {
         roller.move_voltage(-12000);                   //Bota solo roller
         intake.move_voltage(0);
     }

     else if (RightBumperDOWN.isPressed())              //Hold button to Chupa Intake + Roller
     {
         intake.move_voltage(-12000);
         roller.move_voltage(-12000);
         while (RightBumperUP.isPressed())
         {
             intake.move_voltage(0);
             roller.move_voltage(-12000);               //Bota solo roller
         }
     }

     else if (RightBumperUP.isPressed())                //Hold button to Bota Intake + Roller
     {
         intake.move_voltage(12000);
         roller.move_voltage(12000);

           while(RightBumperDOWN.isPressed())
           {
               intake.move_voltage(0);
               roller.move_voltage(-12000);             //Bota solo roller
           }
     }

     else                                               //Intake stops if nothing is pressed
     {
         intake.move_voltage(0);
         roller.move_voltage(0);
     }
     ////////////////////////////////INDEXER///////////////////////////////////

          if (LeftBumperUP.isPressed())                //Hold button to use indexer
          {
              indexer.move_voltage(-12000);
          }
          else if (LeftBumperDOWN.isPressed())                //Hold button to use indexer
          {
              indexer.move_voltage(12000);
          }
          else
          {
              indexer.move_voltage(0);
          }

      ////////////////////////////////FLYWHEEL//////////////////////////////////
      gain = 0.0013;
      if (ButtonRIGHT.changedToPressed())          //Flywheel MID conditions
      {
              FwVelocitySet(FLYWHEEL_MID, 1);
      }

      else if (ButtonUP.changedToPressed())             //Flywheel HIGH conditions
      {
              FwVelocitySet(FLYWHEEL_HIGH, 1);
      }

      else if (ButtonLEFT.changedToPressed())
      {
              FwVelocitySet(0, 1);
      }

		 /////////////////////////////////DESCORER//////////////////////////////////
     descorer.move_voltage(12000*controller.getAnalog(ControllerAnalog::rightY));


		 pros::Task::delay(20);   //opcontrol loop speed
	 }
}
