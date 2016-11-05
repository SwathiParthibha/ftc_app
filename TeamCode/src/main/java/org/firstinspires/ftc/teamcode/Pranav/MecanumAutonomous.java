/* Copyright (c) 2015 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.Pranav;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MecanumAutonomous", group = "Mecanum")
//@Disabled

/*
This is an example of a Basic Autonomous utilizing the functions from RobotHardware Class
 */

public class MecanumAutonomous extends LinearOpMode
{

  DcMotor frontRight;
  DcMotor backRight;
  DcMotor frontLeft;
  DcMotor backLeft;

  //Where all Sensors are defined
  ModernRoboticsI2cGyro gyro = null;
  // public LightSensor legoLineSensor = null;
  // public ModernRoboticsI2cRangeSensor rangeSensor = null;

  //1000 Milliseconds
  int SECOND = 1000;

  //Ultra Sonic Distance
  int ultraSonicDistance = 11;

  //Motor Variables
  int ROTATION = 1220; // # of ticks
  int MOTOR_POWER = 1;

  //Z-Axis of the Modern Robotics Gyro Sensor
  int heading = 0;


  //This function sets the motors to 0 stopping the Robot
  public void stopRobot()
  {
    frontRight.setPower(0);
    backRight.setPower(0);
    frontLeft.setPower(0);
    backLeft.setPower(0);
  }

  public void stopAndResetEncoders()
  {
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void runToPosition()
  {
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void runUsingEncoders()
  {
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  //A basic go straight function that stops after a certain time
  public void goStraight(double Speed, long Time) throws InterruptedException
  {
    frontRight.setPower(MOTOR_POWER * Speed);
    backRight.setPower(MOTOR_POWER * Speed);
    frontLeft.setPower(MOTOR_POWER * Speed);
    backLeft.setPower(MOTOR_POWER * Speed);

    Thread.sleep(Time);
  }

  //A basic go straight function that uses encoders to track its distance
  public void drive(int distance, double speed) throws InterruptedException
  {
    telemetry.addData("Starting to Drive", frontRight.getCurrentPosition() / ROTATION);
    telemetry.update();

    runUsingEncoders();

    stopAndResetEncoders();

    frontRight.setPower(MOTOR_POWER * speed);
    backRight.setPower(MOTOR_POWER * speed);
    frontLeft.setPower(MOTOR_POWER * speed);
    backLeft.setPower(MOTOR_POWER * speed);

    frontRight.setTargetPosition(distance);
    backRight.setTargetPosition(distance);
    frontLeft.setTargetPosition(distance);
    backLeft.setTargetPosition(distance);

    runToPosition();

    while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && opModeIsActive() )
    {

    }

    stopRobot();

    telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
    telemetry.update();
  }

  //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
  public void turnGyro(String Direction, int angle, double Speed) throws InterruptedException
  {
    int MotorDirectionChange = 0;

    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    if (Direction.equals("Left"))
    {
      MotorDirectionChange = -1;
    }
    else
    if (Direction.equals("Right"))
    {
      MotorDirectionChange = 1;
    }

    while ((heading > angle + 5 || heading < angle - 2))
    {
      frontRight.setPower(MOTOR_POWER * Speed * MotorDirectionChange);
      backRight.setPower(MOTOR_POWER * Speed * MotorDirectionChange);

      frontLeft.setPower(-MOTOR_POWER * Speed * MotorDirectionChange);
      backLeft.setPower(-MOTOR_POWER * Speed * MotorDirectionChange);

      heading = gyro.getHeading();
    }

    stopRobot();

  }

  @Override
  public void runOpMode() throws InterruptedException
  {

    // Define and Initialize Motors
    frontRight = hardwareMap.dcMotor.get("motor_2");
    backRight = hardwareMap.dcMotor.get("motor_4");
    frontLeft = hardwareMap.dcMotor.get("motor_3");
    backLeft = hardwareMap.dcMotor.get("motor_1");

    //Define and Initialize Sensors
    gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    // legoLineSensor = hardwareMap.lightSensor.get("legoLineSensor");
    // rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

    //Configure the Direction of the Motors
    frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
    backRight.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
    frontLeft.setDirection(DcMotor.Direction.FORWARD);
    backLeft.setDirection(DcMotor.Direction.FORWARD);

    // Set all motors to zero power
    frontRight.setPower(0);
    backRight.setPower(0);
    frontLeft.setPower(0);
    backLeft.setPower(0);

    // Set all motors to run without encoders.
    // May want to use RUN_USING_ENCODERS if encoders are installed.
    runUsingEncoders();

    waitForStart();

    drive((ROTATION *2), MOTOR_POWER * 0.5);


  }
}