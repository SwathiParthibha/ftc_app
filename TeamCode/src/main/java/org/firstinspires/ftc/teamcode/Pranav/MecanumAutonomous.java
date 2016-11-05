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

@Autonomous(name = "Mecanum Autonomous", group = "Sensor")
//@Disabled

/*
This is an example of a Basic Autonomous utilizing the functions from RobotHardware Class
 */
public class MecanumAutonomous extends LinearOpMode
{
  MecanumHardware mecanum = new MecanumHardware();

  @Override
  public void runOpMode() throws InterruptedException
  {
    // Define and Initialize Motors
    mecanum.frontRight = hardwareMap.dcMotor.get("motor_2");
    mecanum.backRight = hardwareMap.dcMotor.get("motor_4");
    mecanum.frontLeft = hardwareMap.dcMotor.get("motor_3");
    mecanum.backLeft = hardwareMap.dcMotor.get("motor_1");

    //Define and Initialize Sensors
    mecanum.gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    // legoLineSensor = hwMap.lightSensor.get("legoLineSensor");
    // rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

    //Configure the Direction of the Motors
    mecanum.frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
    mecanum.backRight.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
    mecanum.frontLeft.setDirection(DcMotor.Direction.FORWARD);
    mecanum.backLeft.setDirection(DcMotor.Direction.FORWARD);

    // Set all motors to zero power
    mecanum.stopRobot();

    // Set all motors to run using encoders.
    // May want to use RUN_WITHOUT_ENCODER if encoders are not installed.
    mecanum.runUsingEncoder();

    //Calibrate the Modern Robotics Gyro Sensor
    mecanum.gyro.calibrate();

    //Turn on the LED of the Lego Line Sensor
    //legoLineSensor.enableLed(true);

    // wait for the start button to be pressed.
    waitForStart();
    {
      while (mecanum.gyro.isCalibrating())
      {
        Thread.sleep(50);
      }

      //An example of using the Drive function from the RobotHardware Class
      mecanum.drive(mecanum.ROTATION * 2, 0.5);

      //A little bit of settling time
      sleep(500);

      //mecanum.turnGyro("right",45, 0.5);

      sleep(500);
    }

    while(true)
    {
      //Getting the Lego Line Sensor Values
      //telemetry.addData("Light: %d", robot.LegoLineSensor.getLightDetected());*
      telemetry.update();
    }
  }
}