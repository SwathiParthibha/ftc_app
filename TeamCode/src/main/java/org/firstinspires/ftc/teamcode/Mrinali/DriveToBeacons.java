/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Mrinali;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mrinali.HardwarePushbot;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "light sensor")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Beacons Autonomous", group="Pushbot")
//@Disabled
public class DriveToBeacons extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.Mrinali.HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cRangeSensor sideRangeSensor;
    ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    int angleZ = 0;

    // get a reference to a Modern Robotics GyroSensor object.

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    double DIST = 8;
    double SIDE_DIST = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");                // Primary LEGO Light Sensor
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        sideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "side range");
        angleZ  = gyro.getIntegratedZValue();
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle();
        }

        toWhiteLine(false);

        // Turn right - to beacon
        robot.leftMotor.setPower(-APPROACH_SPEED);
        while (opModeIsActive() && (angleZ < 90)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Angle", angleZ);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0); // REPLACE: Use gyro

        approachBeacon();
        pushButton();

        // Go backwards slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);

        // Turn left - parallel to wall
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(APPROACH_SPEED);
        while (opModeIsActive() && (angleZ > 0)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Angle", angleZ);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        maintainDist();

        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);
        sleep(750);

        toWhiteLine(true);

        //Turn right - to beacon
        robot.leftMotor.setPower(-APPROACH_SPEED);
        while (opModeIsActive() && (angleZ < 90)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Angle", angleZ);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);

        approachBeacon();
        pushButton();
    }

    void toWhiteLine(boolean wall) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            if (wall) {
                maintainDist();
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // Stop all motors

        sleep(100);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void approachBeacon() {
        // Drive to set distance away, slow down, stop at set distance
        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }

        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }

        //Momentarily stop
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sleep(200);

        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .25);
            robot.rightMotor.setPower(APPROACH_SPEED * .25);
        }

        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void pushButton() {
        // Pushes button, then straightens
        // REPLACE: Code to push button, use color sensor
        sleep(250);
        robot.rightMotor.setPower(APPROACH_SPEED * .5);
        sleep(500); // REPLACE: Use gyro
        robot.rightMotor.setPower(-APPROACH_SPEED * .5);
        sleep(500); // REPLACE: Use gyro
    }

    void maintainDist() {

        telemetry.addData("Side Range: ", sideRangeSensor.getDistance(DistanceUnit.CM) );
        telemetry.update();

        // If too close to wall, turn right
        if (sideRangeSensor.getDistance(DistanceUnit.CM) < SIDE_DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED * .5);
        }

        // If too far from wall, turn left
        else if (sideRangeSensor.getDistance(DistanceUnit.CM) > SIDE_DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .5);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }

        else {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }
    }
}