#include "definitions.hpp"

// TBH control algorithm variables (global)
long            target_velocity;        // target_velocity velocity
float           current_error;          // error between actual and target_velocity velocities
float           last_error;             // error last time update called
float           gain;                   // gain
float           drive;                  // final drive out of TBH (0.0 to 1.0)
float           drive_at_zero;          // drive at last zero crossing
long            first_cross;            // flag indicating first zero crossing
float           drive_approx;           // estimated open loop drive
float           motor_velocity;         // current velocity in rpm
long            motor_drive;            // final motor control value

//Set the controller position
//desired velocity
//predicted_drive estimated open loop motor drive //Set to 1
void FwVelocitySet( int vel, float predicted_drive )
{
  // set target_velocity velocity (motor rpm)
  target_velocity = vel;

  // Set error so zero crossing is correctly detected
  current_error = target_velocity - motor_velocity;
  last_error    = current_error;

  // Set predicted open loop drive value
  drive_approx  = predicted_drive;
  // Set flag to detect first zero crossing
  first_cross   = 1;
  // clear tbh variable
  drive_at_zero = 0;
}


//Update the velocity tbh controller variables
void FwControlUpdateVelocityTbh()
{
  // calculate error in velocity
  // target_velocity is desired velocity
  // current is measured velocity
  current_error = target_velocity - motor_velocity;

  // Calculate new control value
  drive =  drive + (current_error * gain);

  // Clip to the range 0 - 1.
  // We are only going forwards
  if( drive > 1 )
        drive = 1;
  if( drive < 0 )
        drive = 0;
  // Check for zero crossing
  if((current_error && last_error > 0)||(current_error && last_error < 0)||(current_error && last_error == 0)) //equivalent of: if( sgn(current_error) != sgn(last_error) )
  {
      // First zero crossing after a new set velocity command
      if( first_cross ) {
          drive = drive_approx;       // Set drive to the open loop approximation
          first_cross = 0;
      }
      else
          drive = 0.5 * ( drive + drive_at_zero );
          drive_at_zero = drive;      // Save this drive value in the "tbh" variable
  }
  last_error = current_error;         // Save last error
}



//Task to control the velocity of the flywheel////////////////////////////////////////////////////////////
void FwControlTask(void* param)
{
  // Set the gain
  gain = 0.0013;   // Test with 0.0005, 0.002, 0.0015
  while(1)
      {
      // Calculate velocity
      motor_velocity = flywheel.getActualVelocity();

      // Do the velocity TBH calculations
      FwControlUpdateVelocityTbh() ;

      // Scale drive into the range the motors need
      motor_drive  = (drive * FW_MAX_POWER) + 0.5;

      // Final Limit of motor values - don't really need this
      if( motor_drive >  12000 ) motor_drive =  12000;
      if( motor_drive < -12000 ) motor_drive = -12000;

      // and finally set the motor control value
      flywheel.move_voltage(motor_drive);

      // Run at somewhere between 20 and 50mS
      pros::Task::delay(FW_LOOP_SPEED);
      }
  }


//PID turn with gyro
void PIDGyroTurn(int target, QTime waitTime, float maxPower = 0.8, float Kp = 0.00028, float Ki = 0, float Kd = 0)  //Estos valores eran para cuando los motores iban a 127.
 {
   //	float Kp = 0.2;             // Para girar con el brazo abajo sin peso: Kp = 0.2, Ki = 0.05, Kd = 0.5
   //	float Ki = 0.05;            // Para girar con el brazo a mitad sin peso: Kp = 0.15, Ki = 0.075, Kd = 0.6
   //	float Kd = 0.5;             // Se puede hacer una funcion que dependa de los potenciometros del brazo para ajustar las variables del PID si es necesario^^

   float error;
   float proportion;
   int integralRaw;
   float integral;
   int lastError;
   int derivative;

   float integralActiveZone = 5;        // Valor del gyro cerca del target para que empieze a trabajar el integral
   float integralPowerLimit = 0.5;       // Limite de power que utiliza el integral (ocurre cuando el robot esta en el integralActiveZone) SE NECESITA TUNING (cambiar el 50)
   float finalPower;

   Timer timer;
   timer.placeMark();
   while (timer.getDtFromMark() < waitTime) //Mientras el timer este menor que
   {

     error = target-((((-1*gyro2.get())+gyro1.get())/2));                                                         // P
     proportion = Kp*error;

     if (abs(error) < integralActiveZone && (error != 0))   //markparentheses                                  // I
     {
       integralRaw = integralRaw+error;
     }
     else
     {
       integralRaw = 0;
     }

     if (integralRaw > integralPowerLimit)
     {
       integralRaw = integralPowerLimit;            //integral power limit si vira a valor + del gyro
     }
     if (integralRaw < -integralPowerLimit)
     {
       integralRaw = -integralPowerLimit;           //integral power limit si vira a valor + del gyro
     }

     integral = Ki*integralRaw;


     derivative = Kd*(error-lastError);                                                                           // D
     lastError = error;

     if (error == 0)
     {
       derivative = 0;
     }

     finalPower = proportion+integral+derivative;     //PID SUM

     if (finalPower > maxPower)             //Power limit R
     {
       finalPower = maxPower;
     }
     else if (finalPower < -maxPower)       //Power limit L
     {
       finalPower = -maxPower;
     }

     printf("error = %f, final power = %f\n", error, finalPower);
     driveController.rotate(-finalPower);

     pros::delay(20);

     if (error > 3) //Error depende #tuning              //Cuando entra a la zona cerca del valor que quieres
     {
                                                    //comienza a correr el timer para que no este en
          timer.clearMark();
          timer.placeMark();
     }
   }
   driveController.stop();
 }

//Removes slack before turn. Corrects with gyro at the end.
void Noslackturn(int degrees, int gyroTarget, int gyroThreshold = 50)  //Tune threshold
{
 if (degrees > 0)
 {
   driveController.right(0.5);         //Remove Slack
   driveController.left(-0.5);
   pros::delay(75);
   driveController.turnAngle(degrees);  //Do movement
   if (((((-1*gyro2.get())+gyro1.get())/2) > (gyroTarget+gyroThreshold)) || ((((-1*gyro2.get())+gyro1.get())/2) < (gyroTarget-gyroThreshold)))
   //If 2 gyros average is out of threshold
   {
     //gyroTarget ----> Target for GYRO PID
     //FIX TURN WITH GYRO PID
     //change once I have the tuning values
     //PIDGyroTurn (int target, QTime waitTime, float maxPower = 1, float Kp = 0.0003, float Ki = 0.075, float Kd = 0.6)
   }
 }
 else
 {
   driveController.right(-0.5);         //Remove Slack
   driveController.left(0.5);
   pros::delay(75);
   driveController.turnAngle(degrees);  //Do movement
   if (((((-1*gyro2.get())+gyro1.get())/2) > (gyroTarget+gyroThreshold)) || ((((-1*gyro2.get())+gyro1.get())/2) < (gyroTarget-gyroThreshold)))
   //If 2 gyros average is out of threshold
   {
     //gyroTarget ----> Target for GYRO PID
     //FIX TURN WITH GYRO PID
     //change once I have the tuning values
     //PIDGyroTurn (int target, QTime waitTime, float maxPower = 1, float Kp = 0.0003, float Ki = 0.075, float Kd = 0.6)
   }
 }
}


//Move forward after removing gear/chain slack
void Noslackmove(int distance)
{
  if (distance > 0)
  {
    driveController.right(0.5);         //Remove Slack
    driveController.left(0.5);
    pros::delay(75);
    driveController.moveDistance(distance*foot);  //Do movement
  }
  else
  {
    driveController.right(-0.5);         //Remove Slack
    driveController.left(-0.5);
    pros::delay(75);
    driveController.moveDistance(distance*foot);  //Do movement
  }
}


void alignStep(int vel, int line) {
  bool seenLineR = false;
  bool seenLineL = false;
  driveControllerR.setTarget(vel);
  driveControllerL.setTarget(vel);
  while (!seenLineR || !seenLineL) {
    if (linetrackerR.get_value() < line && !seenLineR) {
      seenLineR = true;
      printf("LINE RIGTH at vel %f\n", driveR.getActualVelocity());
    }
    if (linetrackerL.get_value() < line && !seenLineL) {
      seenLineL = true;
      printf("LINE LEFT at vel %f\n", driveL.getActualVelocity());
    }
    if (seenLineR) {
      driveControllerR.setTarget(0);
    }
    if (seenLineL) {
      driveControllerL.setTarget(0);
    }
  }
  driveControllerR.setTarget(0);
  driveControllerL.setTarget(0);
}

void alignWithLine(int vel, int line, int alignSteps) {
  alignStep(vel, line);
  int fixvel = vel > 0 ? 30 : -30;
  for (int i = 0; i < alignSteps; i++) {
    if (!i%2) {
      fixvel = -fixvel;
    }
    alignStep(fixvel, line);
  }
}


//DELETE IF NEW ONE WORKS
  void lineFW_OLD(float movepower, float fixpower, int line = 900)   //Powers are from 1 to -1
  {
    movepower = movepower*0.01;
    fixpower = fixpower*0.01;
    driveController.forward(movepower);
    while((linetrackerL.get_value() > line) || (linetrackerR.get_value() > line))
  	{

      	while((linetrackerL.get_value() > line) && (linetrackerR.get_value() < line))
      	{
      			driveController.right(-fixpower*0.1);
            driveController.left(fixpower);
      	}
        //
        while((linetrackerL.get_value() < line) && (linetrackerR.get_value() > line))
      	{
          driveController.right(fixpower);
          driveController.left(-fixpower*0.1);
      	}
  	}
    driveController.stop();
  }

//TODO: try putting linetracker much higher on robot for better range of values and distinction between red and blue lines.
  void lineFW_NEW(float movepower, float fixpower, int line)                      //movepower and fixpower are in %
  {
    bool leftDetection = false;
    bool rightDetection = false;
    movepower = movepower*0.01;
    fixpower = fixpower*0.01;
    driveControllerL.setTarget(60);                                                 //Move left side
    driveControllerR.setTarget(60);                                                 //Move left side
    while (rightDetection == false  || leftDetection == false)        //While the line is not being detected by any side
    {

      if ((linetrackerR.get_value() < line) && (rightDetection == false))       //if line is detected by right side
      {
        rightDetection = true;
        printf("DETECTED RIGHT: %.d\n", rightDetection);
      }
      if ((linetrackerL.get_value() < line) && (leftDetection == false))       //if line is detected by right side
      {
        leftDetection = true;
        printf("-------------DETECTED LEFT: %.d\n", leftDetection);
      }
      if ((rightDetection == true) && (leftDetection == false))       //if line is detected by left side
      {
        driveControllerR.setTarget(0);                                                  //Brake right side
        driveControllerL.setTarget(40);
        if(linetrackerR.get_value() > line )
        {
          rightDetection = false;
          driveControllerR.setTarget(-30);
        }                                                 //Move left side
      }
      else if ((leftDetection == true) && (rightDetection == false))  //if line is detected by left side
      {
        driveControllerL.setTarget(0);                                                  //Brake left side
        driveControllerR.setTarget(40);                                                 //Move right side
        if(linetrackerL.get_value() > line )
        {
          leftDetection = false;
          driveControllerL.setTarget(-30);
        }
      }
    }
    driveControllerL.setTarget(0);                                                 //Move left side
    driveControllerR.setTarget(0);                                                 //Move left side
    // //MAY OR MAY NOT BE NECESSARY
    // if (driveControllerL.isDisabled() == false)                                         //If left drive controller is enabled, disable it
    // {
    //   driveControllerL.flipDisable();
    // }
    // if (driveControllerR.isDisabled() == false)                                         //If right drive controller is enabled, disable it
    // {
    //   driveControllerR.flipDisable();
    // }
                                             //Stop
  }


  // int  lineL;
  // bool pgfLeft;
  // bool pgbLeft;
  // int  current_speedL;
  // int threshold = 500;
  // bool endTaskL;
  //bool direction;
  // void LeftCorrect(void*param)
  // {
  //   lineL = 900;
  //   pgfLeft = false;
  //   pgbLeft = false;
  //   current_speedL = 5000;
  //   endTaskL = false;
  //   while(endTaskL == false)
  //   {
  //     printf("pgfLeft: %d\n", pgfLeft);
  //     printf("-------------pgbLeft: %d\n", pgbLeft);
  //     printf("vel: %.f\n", driveL.getActualVelocity());
  //
  //     baseL.moveVoltage(current_speedL);
  //
  //     if (linetrackerL.get_value() <= lineL)
  //     {
  //         if(driveL.getActualVelocity() > 0)
  //         {
  //           pgfLeft = true;
  //           pgbLeft = false;
  //         }
  //         else if (driveL.getActualVelocity() < 0)
  //         {
  //           pgfLeft = false;
  //           pgbLeft = true;
  //         }
  //     }
  //     if (linetrackerL.get_value() > lineL)
  //     {
  //       if(pgfLeft == true)
  //       {
  //         if (driveL.getActualVelocity() > 0)
  //         {
  //           current_speedL = -(abs(current_speedL/2));
  //         }
  //         else if (driveL.getActualVelocity() < 0)
  //         {
  //         current_speedL = abs(current_speedL/2);
  //         }
  //       }
  //       else if(pgbLeft == true)
  //         {
  //             if (driveL.getActualVelocity() < 0)
  //             {
  //               current_speedL = abs(current_speedL/2);
  //               pgbLeft = false;
  //             }
  //             else if (driveL.getActualVelocity() > 0)
  //             {
  //             current_speedL = abs(current_speedL/2);
  //               pgbLeft = false;
  //             }
  //         }
  //       }
  //     if (current_speedL < threshold) // || driveL.getActualVelocity() = 0
  //     {
  //         endTaskL = true;
  //     }
  //     pros::Task::delay(20);
  //   }
  // }
