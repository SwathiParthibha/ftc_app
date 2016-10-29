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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Mecanum Autonomous", group = "Autonomous")
//@Disabled

/*
This is an example of a Basic Autonomous utilizing the functions from RobotHardware Class
 */

public class Mecanum_Go_Straight extends LinearOpMode
{
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft ;
    DcMotor backLeft;

    int ROTATION = 1220;
    int SECOND = 1000;

    int MOTOR_POWER = 1;

    public void stopRobot()
    {
        frontRight.setPower(MOTOR_POWER * 0);
        backRight.setPower(MOTOR_POWER * 0);
        frontLeft.setPower(MOTOR_POWER * 0);
        backLeft.setPower(MOTOR_POWER * 0);
    }

    public void drive(int Distance, double Speed) throws InterruptedException
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setPower(MOTOR_POWER * Speed);
        backRight.setPower(MOTOR_POWER * Speed);
        frontLeft.setPower(MOTOR_POWER * Speed);
        backLeft.setPower(MOTOR_POWER * Speed);

        frontRight.setTargetPosition(Distance);
        backRight.setTargetPosition(Distance);
        frontLeft.setTargetPosition(Distance);
        backLeft.setTargetPosition(Distance);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())
        {

        }

        stopRobot();

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

  @Override
  public void runOpMode() throws InterruptedException
  {
      frontRight = hardwareMap.dcMotor.get("motor_2");
      backRight = hardwareMap.dcMotor.get("motor_4");
      frontLeft = hardwareMap.dcMotor.get("motor_3");
      backLeft = hardwareMap.dcMotor.get("motor_1");

      frontRight.setDirection(DcMotor.Direction.FORWARD);
      backRight.setDirection(DcMotor.Direction.FORWARD);
      frontLeft.setDirection(DcMotor.Direction.REVERSE);
      backLeft.setDirection(DcMotor.Direction.REVERSE);

      // Set all motors to zero power
      frontRight.setPower(0);
      backRight.setPower(0);
      frontLeft.setPower(0);
      backLeft.setPower(0);

      // Set all motors to run without encoders.
      // May want to use RUN_USING_ENCODERS if encoders are installed.
      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


      waitForStart();
      {
          drive(ROTATION * 2, 1.0);
      }
  }
}