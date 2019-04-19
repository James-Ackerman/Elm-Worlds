#include "main.h"

void autonomous() {
//Initial auton settings
pros::lcd::set_text(7, "AUTONOMOUS");
pros::Task FwControl(FwControlTask);
flywheel.setGearing(AbstractMotor::gearset::blue);
/////////////////////////////AUTONOMOUS RED//////////////////////////////////////////////////////////////////////////////////////////////////////////
FwVelocitySet(600, 1);
pros::delay(300);
FwVelocitySet(400, 1);
pros::delay(1000);
//
//No slack move async with raise lift
driveController.setMaxVelocity(70);
driveController.right(0.5);
driveController.left(0.5);
pros::delay(75);
driveController.moveDistanceAsync(2.3_ft);
descorerController.setTarget(260);
pros::delay(200);
shoot1Ball();
FwVelocitySet(0, 1);
driveController.waitUntilSettled();
pros::delay(130);
//

//No slack move with lower lift BEGINNING
driveController.setMaxVelocity(55);
driveController.right(0.5);
driveController.left(0.5);
pros::delay(75);
driveController.moveDistanceAsync(-2.8_ft);
descorerController.setTarget(200);
driveController.waitUntilSettled();

pros::delay(100);
FwVelocitySet(600, 1);
Noslackturn(40.5, 30);
pros::delay(100);
Noslackmove(1.6, 40);
descorerController.setTarget(105);

//No slack move Async with chupa intake
driveController.setMaxVelocity(80);
driveController.right(0.5);
driveController.left(0.5);
pros::delay(75);
driveController.moveDistanceAsync(-0.3_ft);
pros::delay(100);

roller.move_voltage(12000);
intake.move_voltage(12000);
indexer.move_voltage(500);
driveController.waitUntilSettled();
pros::delay(200);
descorerController.setTarget(200);
Noslackmove(-0.5, 30);
pros::delay(200);
descorerController.setTarget(0);
pros::delay(150);
Noslackmove(1.15, 30);
//Sube Cap
descorerController.setMaxVelocity(50);
descorerController.setTarget(200);

Noslackturn(10, 40);

pros::delay(500);
shoot2Balls();
FwVelocitySet(0, 1);
roller.move_voltage(0);
intake.move_voltage(0);
indexer.move_voltage(0);

//Leave cap for Dingo
Noslackturn(155, 60);
Noslackmove(0.85, 50);
descorerController.setMaxVelocity(100);
descorerController.setTarget(0);
descorerController.waitUntilSettled();
driveController.setMaxVelocity(80);
Noslackmove(-1.2, 50);
descorerController.setTarget(260);

Noslackturn(-38, 60);
Noslackmove(2.55, 45);

//intake ball on platform
descorerController.setTarget(100);
roller.move_voltage(12000);
intake.move_voltage(12000);
Noslackmove(-0.5, 60);
indexer.move_voltage(500);
pros::delay(500);
roller.move_voltage(0);
intake.move_voltage(0);
indexer.move_voltage(0);
descorerController.setTarget(50);
FwVelocitySet(535, 1);

//Turn for cap
Noslackturn(-80, 60);
descorerController.setTarget(260);
roller.move_voltage(12000);
intake.move_voltage(12000);
Noslackmove(1.5, 60);
indexer.move_voltage(400);
pros::delay(1000);
roller.move_voltage(0);
intake.move_voltage(0);
Noslackmove(-0.4, 60);
Noslackturn(-95, 60);

//Acomoda con plat
driveController.forward(-0.4);
pros::delay(700);
driveController.stop();
Noslackmove(0.5, 30);
pros::delay(75);

//Le da al flag de abajo
Noslackturn(12, 30);
pros::delay(75);
Noslackmove(3.5, 50);
descorerController.setTarget(0);
FwVelocitySet(520, 1);
Noslackmove(-0.38, 45);
pros::delay(75);
Noslackturn(64.5, 30);
shoot2Balls();

//Tira
// indexer.move_voltage(-12000);
// intake.move_voltage(12000);
// roller.move_voltage(12000);
// pros::delay(700);
// indexer.move_voltage(0);
// intake.move_voltage(0);
// roller.move_voltage(0);
// FwVelocitySet(0, 1);







//
////////////////////////////Reference stuff///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //FwVelocitySet(490, 1);
  // pros::delay(500);
  // intake.moveVoltage(-12000);
  // indexer.moveVoltage(1000);
  // roller.moveVoltage(12000);
  // driveController.moveDistance(-1_ft);    //move
  // driveController.turnAngle(90_deg);        //turn
  // driveController.setMaxVoltage(7000);      //set max Voltage of controller
  // PIDGyroTurn(150, 600_ms, 1.0, 0.0065, 0.06, 0.05);
  // state != state;                           //flip bool for pneumatics
  // sensor.set_value(state);                  //set piston va
  // rotatorController.setTarget(180_deg);     //flip intake
  // conveyorController.setTarget(200_rpm);    //move conveyor full speed
  // liftController.setTarget(50_deg);         //raise lift //TODO: measure distance per degree
  // driveController.moveDistanceAsync(1_m);   // Move 1 meter to the first goal
  //driveController.setMaxVoltage(800);
  //driveController.setMaxVelocity(60);
  //pros::delay(1000) or pros::Task::delay(1000)
  //Noslackturn(92.72, 150);
  //NoslackturnGyro(92.72, 150, 5);
  //Noslackmove(2, 50);
  //driveController.setMaxVoltage(8000)

  //Line Trackers
  //alignWithLine(100, 900, 2); // WORKING FOR VELOCITY < 100, LINE = 900, alignSteps = 2

  // Testing Timers
  // Timer timer;
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
