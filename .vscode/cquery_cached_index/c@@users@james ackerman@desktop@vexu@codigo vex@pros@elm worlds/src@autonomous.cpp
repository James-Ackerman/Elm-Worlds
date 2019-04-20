#include "main.h"

void autonomous() {
//Initial auton settings
pros::lcd::set_text(7, "AUTONOMOUS");
pros::Task FwControl(FwControlTask);
flywheel.setGearing(AbstractMotor::gearset::blue);
descorerController.tarePosition();
/////////////////////////////AUTONOMOUS BLUE FRONT//////////////////////////////////////////////////////////////////////////////////////////////////////////
// FwVelocitySet(600, 1);
// pros::delay(300);
// FwVelocitySet(380, 1);
// pros::delay(1000);
// //
// //No slack move async with raise lift
// driveController.setMaxVelocity(40);
// driveController.right(0.5);
// driveController.left(0.5);
// pros::delay(75);
// driveController.moveDistanceAsync(2.3_ft);
// descorerController.setTarget(260);
// pros::delay(300);
// shoot1Ball();
// FwVelocitySet(0, 1);
// driveController.waitUntilSettled();
// pros::delay(130);
// //
//
// //No slack move with lower lift BEGINNING
// driveController.setMaxVelocity(40);
// driveController.forward(-0.5);
// pros::delay(75);
// driveController.moveDistanceAsync(-2.8_ft);
// FwVelocitySet(590, 1);
// descorerController.setTarget(200);
// driveController.waitUntilSettled();
//
// pros::delay(100);
//
// //Coge Bolitas encima  cap
// Noslackturn(40.5, 30);
// pros::delay(100);
// Noslackmove(1.6, 40);
// descorerController.setTarget(105);
// //No slack move Async with chupa intake
// driveController.setMaxVelocity(80);
// driveController.right(0.5);
// driveController.left(0.5);
// pros::delay(75);
// driveController.moveDistanceAsync(-0.3_ft);
// pros::delay(100);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// indexer.move_voltage(500);
// driveController.waitUntilSettled();
// pros::delay(200);
// descorerController.setTarget(200);
// Noslackmove(-0.5, 30);
// pros::delay(200);
// descorerController.setTarget(0);
// pros::delay(150);
// Noslackmove(1.15, 30);
// //Sube Cap
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(260);
//
// Noslackturn(10.7, 30);
// //SHoot 2 balls
// shoot1Ball();
// pistonS.set_value(HIGH);
// FwVelocitySet(600, 1);
// pros::delay(1000);
// shoot1Ball();
// pistonS.set_value(LOW);
// FwVelocitySet(0, 1);
// roller.move_voltage(0);
// intake.move_voltage(0);
// indexer.move_voltage(0);
//
// //Leave cap for Dingo
// Noslackturn(155, 60);
// Noslackmove(0.85, 50);
// descorerController.setMaxVelocity(100);
// descorerController.setTarget(0);
// descorerController.waitUntilSettled();
// driveController.setMaxVelocity(80);
// Noslackmove(-1.2, 50);
// descorerController.setTarget(260);
//
// Noslackturn(-38, 60);
// Noslackmove(2.55, 45);
//
// //intake ball on platform
// descorerController.setTarget(95);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// Noslackmove(-0.5, 60);
// indexer.move_voltage(500);
// pros::delay(700);
// roller.move_voltage(0);
// intake.move_voltage(0);
// indexer.move_voltage(0);
// descorerController.setTarget(50);
// FwVelocitySet(535, 1);
//
// //Turn for cap
// Noslackturn(-80, 60);
// descorerController.setTarget(260);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// Noslackmove(1.5, 60);
// indexer.move_voltage(400);
// pros::delay(1000);
// roller.move_voltage(0);
// intake.move_voltage(0);
// Noslackmove(-0.3, 60);
// Noslackturn(-95, 60);
//
// //Acomoda con plat
// driveController.forward(-0.4);
// pros::delay(750);
// driveController.stop();
// Noslackmove(0.5, 30);
// pros::delay(75);
//
// //Le da al flag de abajo
// Noslackturn(12, 30);
// pros::delay(75);
// Noslackmove(3.5, 50);
// descorerController.setTarget(0);
// FwVelocitySet(520, 1);
// Noslackmove(-0.5, 45);
// pros::delay(75);
// Noslackturn(65, 30);
//
// //Shoot oppo field
// shoot1Ball();
// pistonS.set_value(HIGH);
// FwVelocitySet(600, 1);
// pros::delay(1000);
// shoot1Ball();
// pistonS.set_value(LOW);
// FwVelocitySet(0, 1);

/////////////////////////////AUTONOMOUS BLUE BACK//////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//NEW VERSION
FwVelocitySet(490, 1);
pros::delay(2500);
shoot1Ball();
FwVelocitySet(0, 1);
Noslackmove(-1.6, 30);
descorerController.setTarget(200);
pros::delay(70);
Noslackturn(90, 25);
//Coge Bolitas encima  cap
Noslackmove(2.95, 47);
pros::delay(70);
descorerController.setTarget(105);
descorerController.waitUntilSettled();
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
Noslackmove(-0.5, 40);
pros::delay(200);
descorerController.setTarget(0);
pros::delay(350);
Noslackmove(1.25, 40);

//Acomoda con poste
descorerController.setMaxVelocity(50);
descorerController.setTarget(420);
descorerController.waitUntilSettled();
pros::delay(75);
Noslackturn(15, 40);
Noslackmove(-1.8, 30);
pros::delay(75);
PIDGyroTurn(0.5, 800_ms, 0.3, 0.0040, 0.05, 0.04); //06 int
pros::delay(70);
//Pone Cap
Noslackmove(-0.75, 20);
descorerController.setMaxVelocity(120);
descorerController.setTarget(830);
descorerController.waitUntilSettled();
driveController.stop();

//shoot at middle flags
Noslackmove(0.2, 30);
FwVelocitySet(550, 1);
descorerController.setTarget(260);
descorerController.waitUntilSettled();
PIDGyroTurn(-0.2, 400_ms, 1.0, 0.0060, 0.05, 0.04);
pros::delay(100);
Noslackmove(2.2, 30);
pros::delay(70);
PIDGyroTurn(16.65, 500_ms, 1.0, 0.0060, 0.05, 0.04);

shoot1Ball();
FwVelocitySet(490, 1);
pros::delay(1000);
shoot1Ball();

//intake ball on platform
Noslackmove(0.32, 30);
descorerController.setTarget(100);
roller.move_voltage(12000);
intake.move_voltage(12000);
Noslackmove(-0.7, 50);
descorerController.setTarget(52);
indexer.move_voltage(500);
pros::delay(200);

//intake ball under cap
descorerController.setTarget(260);
PIDGyroTurn(92.72, 600_ms, 1.0, 0.0050, 0.05, 0.04);
pros::delay(200);
Noslackmove(2.3, 50);
pros::delay(700);
//shoot 2 bals to oppo field
FwVelocitySet(575, 1);
Noslackmove(-0.7, 30);
PIDGyroTurn(0, 200_ms, 1.0, 0.0050, 0.05, 0.04);
//Acomoda con plat
driveController.forward(0.2);
pros::delay(750);
driveController.stop();
Noslackmove(-0.6, 30);
pros::delay(75);
Noslackturn(31, 30);
pros::delay(75);
shoot1Ball();
FwVelocitySet(530, 1);
pros::delay(1000);
shoot1Ball();

////////////////////////////Reference stuff///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //swingTurn(200, 0);
  //FwVelocitySet(490, 1);
  // pros::delay(500);
  // intake.moveVoltage(-12000);
  // indexer.moveVoltage(1000);
  // roller.moveVoltage(12000);
  // driveController.moveDistance(-1_ft);    //move
  // driveController.turnAngle(90_deg);        //turn
  // driveController.setMaxVoltage(7000);      //set max Voltage of controller

  //PIDGyroTurn(92.72, 600_ms, 1.0, 0.0060, 0.06, 0.04);

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
