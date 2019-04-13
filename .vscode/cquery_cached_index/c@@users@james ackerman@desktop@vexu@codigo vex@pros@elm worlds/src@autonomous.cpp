#include "main.h"

void autonomous() {
//Initial auton settings
pros::lcd::set_text(7, "AUTONOMOUS");
pros::Task FwControl(FwControlTask);
flywheel.setGearing(AbstractMotor::gearset::blue);
PIDGyroTurn(92.72, 1000_ms, 0.8, 0.0069, 0.045, 0.05);
/////////////////////////////AUTONOMOUS RED//////////////////////////////////////////////////////////////////////////////////////////////////////////
// driveController.forward(0.2);
// pros::delay(200);
// driveController.stop();
//lineFW_OLD(30, 20, 900);                                   //(movepower (%), fixpower(%), line (linetracker sensor value))
//alignWithLine(100, 900, 2); // WORKING FOR VELOCITY < 100, LINE = 900, alignSteps = 2
// pros::Task LeftCorrectTask(LeftCorrect);
// while(endTaskL == false); //&& (endTaskR == true));
// //RightCorrectTask.remove();
// LeftCorrectTask.remove();



//
////////////////////////////Reference stuff///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //FwVelocitySet(490, 1);
  // pros::delay(500);
  // intake.moveVoltage(-12000);
  // indexer.moveVoltage(1000);
  // roller.moveVoltage(12000);
  // driveController.moveDistance(-0.4_ft);    //move
  // driveController.turnAngle(90_deg);        //turn
  // driveController.setMaxVoltage(7000);      //set max Voltage of controller

  // state != state;                           //flip bool for pneumatics
  // sensor.set_value(state);                  //set piston va
  // rotatorController.setTarget(180_deg);     //flip intake
  // conveyorController.setTarget(200_rpm);    //move conveyor full speed
  // liftController.setTarget(50_deg);         //raise lift //TODO: measure distance per degree
  // driveController.moveDistanceAsync(1_m);   // Move 1 meter to the first goal
  //driveController.setMaxVoltage(800);
  //driveController.setMaxVelocity(60);
  //pros::delay(1000) or pros::Task::delay(1000)

  //Using gyro PID
  // while(1)
  // {
  //   GdriveController.setTarget(90);
  // }

  // Testing Timers
  // Timer timer;
  //
  // timer.placeMark();
  // while (timer.getDtFromMark() < 500_ms) //Mientras el timer este menor que
  // {driveController.rotate(0.5);}
  // driveController.stop();

  //Slack removal tuning
  // driveController.right(0.5);         //Remove Slack
  // driveController.left(0.5);
  // pros::delay(75);
  // driveController.moveDistance(2_ft);  //Do movement

  /**
   * If the robot is disabled or communications is lost, the autonomous task
   * will be stopped. Re-enabling the robot will restart the task, not re-start it
   * from where it left off.
   *///
}
