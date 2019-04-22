#include "main.h"

void autonomous() {
//Initial auton settings
pros::lcd::set_text(7, "AUTONOMOUS");
pros::Task FwControl(FwControlTask);
flywheel.setGearing(AbstractMotor::gearset::blue);
descorerController.tarePosition();
pros::delay(50);
/////////////////////////////AUTONOMOUS BLUE FRONT//////////////////////////////////////////////////////////////////////////////////////////////////////////
// FwVelocitySet(600, 1);
// pros::delay(300);
// FwVelocitySet(380, 1);
// pros::delay(1000);
//
// //No slack move async with raise lift
// descorerController.setTarget(260);
// driveController.setMaxVelocity(40);
// driveController.setMaxVelocity(200);
// profileController.generatePath({
//   Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
//   Point{2.5_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
//   "A");
// profileController.setTarget("A", false);
// pros::delay(300);
// shoot1Ball();
// FwVelocitySet(0, 1);
// profileController.waitUntilSettled();
// pros::delay(70);
// //
//
// //No slack move with lower lift BEGINNING
// moveDistance(3.05, true);
// FwVelocitySet(590, 1);
// descorerController.setTarget(200);
// pros::delay(70);
// //Coge Bolitas encima  cap
// PIDGyroTurn(42.7, 700_ms, 1.0, 0.0058, 0.07, 0.055);
// pros::delay(70);
// roller.move_voltage(9000);
// intake.move_voltage(9000);
// moveDistance(1.75, false);
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(129);
//
// //No slack move Async with chupa intake
// driveController.setMaxVelocity(200);
// driveController.forward(-1);
// pros::delay(250);
// driveController.stop();
// descorerController.setTarget(200);
// profileController.generatePath({
//   Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
//   Point{0.4_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
//   "A");
// profileController.setTarget("A", true);
// pros::delay(100);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// indexer.move_voltage(500);
// profileController.waitUntilSettled();
// pros::delay(70);
// descorerController.setTarget(0);
// pros::delay(900);
// moveDistance(1.15, false);
// //Sube Cap
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(260);
//
// //Noslackturn(14, 20);
// PIDGyroTurn(50, 200_ms, 1.0, 0.0070, 0.07, 0.06);
// //SHoot 2 balls
// shoot1Ball();
// pistonS.set_value(HIGH);
// FwVelocitySet(600, 1);
// pros::delay(1200);
// shoot1Ball();
// pistonS.set_value(LOW);
// FwVelocitySet(0, 1);
// roller.move_voltage(0);
// intake.move_voltage(0);
// indexer.move_voltage(0);
//
// //Leave cap for Dingo
// Noslackturn(150, 30);
// moveDistance(0.85, false);
// descorerController.setMaxVelocity(100);
// descorerController.setTarget(0);
// descorerController.waitUntilSettled();
// driveController.setMaxVelocity(80);
// moveDistance(0.7, true);
// descorerController.setTarget(260);
//
// Noslackturn(-38, 60);
// moveDistance(2, false);
// moveDistance(0.2, false);
// //intake ball on platform
// descorerController.setTarget(110);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// moveDistance(0.6, true);
// pros::delay(800);
// FwVelocitySet(535, 1);
//
// //Turn for cap
// Noslackturn(-65, 45);
// descorerController.setTarget(260);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// moveDistance(1.7, false);
// indexer.move_voltage(400);
// pros::delay(1000);
// moveDistance(0.69, true);
// Noslackturn(-85.5, 50);
//
// //Acomoda con plat
// driveController.forward(-0.4);
// pros::delay(800);
// driveController.stop();
// moveDistance(0.5, false);
// roller.move_voltage(0);
// intake.move_voltage(0);

// //Le da al flag de abajo
// Noslackturn(23, 30);
// pros::delay(75);
// moveDistance(3.5, false);
// descorerController.setTarget(0);
// FwVelocitySet(520, 1);
// moveDistance(0.4, true);
// pros::delay(75);
// Noslackturn(61, 30);
//
// //Shoot oppo field
// shoot1Ball();
// pistonS.set_value(HIGH);
// FwVelocitySet(600, 1);
// pros::delay(1000);
// shoot1Ball();
// pistonS.set_value(LOW);
// FwVelocitySet(0, 1);

///////////////////////////AUTONOMOUS BLUE BACK//////////////////////////////////////////////////////////////////////////////////////////////////////////
//FwVelocitySet(600, 1);
//pros::delay(300);
// FwVelocitySet(520, 1);
// pros::delay(2500);
// shoot1Ball();
// FwVelocitySet(545, 1);
// descorerController.setTarget(230);
// moveDistance(1.6, true); //Noslackmove(-1.6, 25);
// pros::delay(70);
// Noslackturn(91.5, 25);
//
// //Coge Bolitas encima cap
// moveDistance(3.1, false); //Noslackmove(3.00, 47);
// pros::delay(150);
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(110);
// descorerController.waitUntilSettled();
// driveController.setMaxVelocity(80);
// driveController.right(0.5);
// driveController.left(0.5);
// pros::delay(75);
// driveController.moveDistanceAsync(-0.45_ft);
// pros::delay(100);
// roller.move_voltage(9000);
// intake.move_voltage(9000);
// driveController.waitUntilSettled();
// pros::delay(200);
// descorerController.setTarget(200);
// moveDistance(0.6, true); //Noslackmove(-0.6, 40);
// pros::delay(200);
// descorerController.setTarget(0);
// pros::delay(450);
// moveDistance(1, false); //Noslackmove(1.25, 40);
//
// //Acomoda con poste
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(420);
// descorerController.waitUntilSettled();
// pros::delay(75);
// Noslackturn(12, 40);
// Noslackmove(-1.2, 30);
// roller.move_voltage(0);
// intake.move_voltage(0);
// pros::delay(75);
// PIDGyroTurn(0.55, 800_ms, 0.3, 0.0046, 0.06, 0.04); //06 int
// pros::delay(70);
//
// //Pone Cap
// driveController.forward(-0.2);
// pros::delay(200);
// driveController.forward(-0.07);
// descorerController.setMaxVelocity(105);
// descorerController.setTarget(830);
// descorerController.waitUntilSettled();
// driveController.stop();
// pros::delay(200);
//
// //shoot at middle flags
// moveDistance(0.2, false);
// descorerController.setTarget(260);
// descorerController.waitUntilSettled();
// PIDGyroTurn(-0.2, 400_ms, 1.0, 0.0060, 0.05, 0.04);
// pros::delay(100);
// roller.move_voltage(7500);
// intake.move_voltage(7500);
// moveDistance(2.2, false);
// pros::delay(70);
// PIDGyroTurn(17, 800_ms, 1.0, 0.0070, 0.06, 0.05);//
//
// shoot1Ball();
// FwVelocitySet(500, 1);
// pros::delay(1000);
// shoot1Ball();
// FwVelocitySet(595, 1);   //575 old
// //intake ball on platform
// moveDistance(0.3, false);
// descorerController.setTarget(100);
// roller.move_voltage(9000);
// intake.move_voltage(9000);
// moveDistance(0.7, true);
// descorerController.setTarget(52);
// pros::delay(200);
//
// //intake ball under cap
// descorerController.setTarget(260);
// PIDGyroTurn(92.72, 600_ms, 1.0, 0.0050, 0.05, 0.04);
// pros::delay(200);
// moveDistance(2.3, false);
// pros::delay(700);
// roller.move_voltage(9000);
// intake.move_voltage(9000);
// descorerController.setTarget(260);
// //shoot 2 bals to oppo field
// moveDistance(0.9, true);
// roller.move_voltage(0);
// intake.move_voltage(0);
// indexer.move_voltage(0);
// PIDGyroTurn(0, 200_ms, 1.0, 0.0050, 0.05, 0.04);
//
// //Acomoda con plat
// driveController.forward(0.2);
// pros::delay(1000);
// driveController.stop();
// pros::delay(750);
// moveDistance(0.5, true);
// pros::delay(75);
// Noslackturn(33, 30);
// pros::delay(75);
// moveDistance(0.3, false);
// shoot1Ball();
// FwVelocitySet(535, 1); //530 old
// pros::delay(1000);
// shoot1Ball();


/////////////////////////////AUTONOMOUS RED FRONT///////////////////////////////////////////////////////////////////////////////////////
// driveController.setTurnsMirrored(true);
// FwVelocitySet(600, 1);
// pros::delay(300);
// FwVelocitySet(380, 1);
// pros::delay(1000);
//
// //No slack move async with raise lift
// descorerController.setTarget(260);
// driveController.setMaxVelocity(40);
// driveController.setMaxVelocity(200);
// profileController.generatePath({
//   Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
//   Point{2.5_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
//   "A");
// profileController.setTarget("A", false);
// pros::delay(300);
// shoot1Ball();
// FwVelocitySet(0, 1);
// profileController.waitUntilSettled();
// pros::delay(70);
//
//
// //No slack move with lower lift BEGINNING
// moveDistance(3.05, true);
// FwVelocitySet(590, 1);
// descorerController.setTarget(200);
// pros::delay(70);
// //Coge Bolitas encima  cap
// PIDGyroTurn(-42.7, 700_ms, 1.0, 0.0058, 0.07, 0.055);
// pros::delay(70);
// roller.move_voltage(9000);
// intake.move_voltage(9000);
// moveDistance(1.75, false);
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(129);
//
// //No slack move Async with chupa intake
// driveController.setMaxVelocity(200);
// driveController.forward(-1);
// pros::delay(220);
// driveController.stop();
// descorerController.setTarget(200);
// profileController.generatePath({
//   Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
//   Point{0.4_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
//   "A");
// profileController.setTarget("A", true);
// pros::delay(100);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// indexer.move_voltage(500);
// profileController.waitUntilSettled();
// pros::delay(70);
// descorerController.setTarget(0);
// pros::delay(900);
// moveDistance(1.15, false);
// //Sube Cap
// descorerController.setMaxVelocity(50);
// descorerController.setTarget(260);
//
// PIDGyroTurn(-50, 200_ms, 1.0, 0.0070, 0.07, 0.06);
//
// //SHoot 2 balls
// shoot1Ball();
// pistonS.set_value(HIGH);
// FwVelocitySet(600, 1);
// pros::delay(1200);
// shoot1Ball();
// pistonS.set_value(LOW);
// FwVelocitySet(0, 1);
// roller.move_voltage(0);
// intake.move_voltage(0);
// indexer.move_voltage(0);
//
// //Leave cap for Dingo
// Noslackturn(145, 30);
// moveDistance(0.85, false);
// descorerController.setMaxVelocity(100);
// descorerController.setTarget(0);
// descorerController.waitUntilSettled();
// driveController.setMaxVelocity(80);
// moveDistance(0.7, true);
// descorerController.setTarget(260);
//
// Noslackturn(-48.3, 60);
// moveDistance(2, false);
// moveDistance(0.2, false);
//
// //intake ball on platform
// descorerController.setTarget(110);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// moveDistance(0.6, true);
// pros::delay(800);
// FwVelocitySet(535, 1);
//
// //Turn for cap
// Noslackturn(-66, 45);
// descorerController.setTarget(260);
// roller.move_voltage(12000);
// intake.move_voltage(12000);
// moveDistance(1.7, false);
// indexer.move_voltage(400);
// pros::delay(1000);
// moveDistance(0.69, true);
// Noslackturn(-85.5, 50);
//
// //Acomoda con plat
// driveController.forward(-0.4);
// pros::delay(800);
// driveController.stop();
// moveDistance(0.5, false);
// roller.move_voltage(0);
// intake.move_voltage(0);
// //Le da al flag de abajo
// Noslackturn(18, 30);
// pros::delay(75);
// moveDistance(3.5, false);
// descorerController.setTarget(0);
// FwVelocitySet(520, 1);
// moveDistance(0.4, true);
// pros::delay(75);
// Noslackturn(55, 30);
//
// //Shoot oppo field
// shoot1Ball();
// pistonS.set_value(HIGH);
// FwVelocitySet(600, 1);
// pros::delay(1000);
// shoot1Ball();
// pistonS.set_value(LOW);
// FwVelocitySet(0, 1);

///////////////////////////AUTONOMOUS RED BACK//////////////////////////////////////////////////////////////////////////////////////////////////////////
driveController.setTurnsMirrored(true);
FwVelocitySet(600, 1);
pros::delay(300);
FwVelocitySet(490, 1);
pros::delay(1500);
shoot1Ball();
FwVelocitySet(545, 1);
descorerController.setTarget(230);
moveDistance(1.6, true); //Noslackmove(-1.6, 25);
pros::delay(70);
Noslackturn(90.2, 25);

//Coge Bolitas encima cap
moveDistance(3.1, false); //Noslackmove(3.00, 47);
pros::delay(150);
descorerController.setMaxVelocity(50);
descorerController.setTarget(110);
descorerController.waitUntilSettled();
driveController.setMaxVelocity(80);
driveController.right(0.5);
driveController.left(0.5);
pros::delay(75);
driveController.moveDistanceAsync(-0.45_ft);
pros::delay(100);
roller.move_voltage(9000);
intake.move_voltage(9000);
driveController.waitUntilSettled();
pros::delay(200);
descorerController.setTarget(200);
moveDistance(0.6, true); //Noslackmove(-0.6, 40);
pros::delay(200);
descorerController.setTarget(0);
pros::delay(450);
moveDistance(1, false); //Noslackmove(1.25, 40);

//Acomoda con poste
descorerController.setMaxVelocity(50);
descorerController.setTarget(420);
descorerController.waitUntilSettled();
pros::delay(75);
Noslackturn(12, 40);
moveDistance(1.27, true);
roller.move_voltage(0);
intake.move_voltage(0);
pros::delay(75);
PIDGyroTurn(-0.52, 600_ms, 0.4, 0.0046, 0.06, 0.04); //06 int
pros::delay(70);

//Pone Cap
driveController.forward(-0.2);
pros::delay(200);
driveController.forward(-0.07);
descorerController.setMaxVelocity(80);
descorerController.setTarget(830);
descorerController.waitUntilSettled();
driveController.stop();
pros::delay(200);

//shoot at middle flags
moveDistance(0.2, false);
descorerController.setTarget(260);
descorerController.waitUntilSettled();
PIDGyroTurn(0.2, 400_ms, 1.0, 0.0060, 0.05, 0.04);
pros::delay(70);
roller.move_voltage(7500);
intake.move_voltage(7500);
moveDistance(2.2, false);
pros::delay(70);
PIDGyroTurn(-17, 600_ms, 1.0, 0.0068, 0.06, 0.05);//

shoot1Ball();
FwVelocitySet(500, 1);
pros::delay(1000);
shoot1Ball();
FwVelocitySet(595, 1);   //575 old
//intake ball on platform
moveDistance(0.3, false);
descorerController.setTarget(100);
roller.move_voltage(9000);
intake.move_voltage(9000);
moveDistance(0.7, true);
descorerController.setTarget(52);
pros::delay(100);

//intake ball under cap
descorerController.setTarget(260);
PIDGyroTurn(-92.72, 300_ms, 1.0, 0.0056, 0.05, 0.05);
pros::delay(100);
moveDistance(2.3, false);
pros::delay(700);
roller.move_voltage(9000);
intake.move_voltage(9000);
descorerController.setTarget(260);
//shoot 2 bals to oppo field
moveDistance(0.9, true);
roller.move_voltage(0);
intake.move_voltage(0);
indexer.move_voltage(0);
PIDGyroTurn(0, 200_ms, 1.0, 0.0050, 0.05, 0.04);

//Acomoda con plat
driveController.forward(0.2);
pros::delay(1000);
driveController.stop();
pros::delay(750);
moveDistance(0.5, true);
pros::delay(75);
PIDGyroTurn(-35.5, 400_ms, 1.0, 0.0060, 0.065, 0.055);
pros::delay(75);
moveDistance(0.3, false);
shoot1Ball();
FwVelocitySet(535, 1); //530 old
pros::delay(1000);
shoot1Ball();

////////////////////////////Reference stuff///////////////////////////////////////////////////////////////////////////////////////////////////////////

// profileController.generatePath({
//   Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
//   Point{0_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
//   "A");
// profileController.setTarget("A");
// profileController.waitUntilSettled();

  //swingTurn(200, 0);
  //FwVelocitySet(490, 1);
  // pros::delay(500);
  // intake.moveVoltage(-12000);
  // indexer.moveVoltage(1000);
  // roller.moveVoltage(12000);
  // driveController.moveDistance(-1_ft);    //move
  // driveController.turnAngle(90_deg);        //turn
  // driveController.setMaxVoltage(7000);      //set max Voltage of controller

  //PIDGyroTurn(92.72, 500_ms, 1.0, 0.0070, 0.07, 0.06);
  //PIDGyroTurn(20, 200_ms, 1.0, 0.0070, 0.07, 0.06);
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
