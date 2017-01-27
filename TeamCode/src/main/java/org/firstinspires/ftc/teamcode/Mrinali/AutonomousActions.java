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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

public class AutonomousActions extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    //HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor  = null;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private boolean state;
    private DcMotor scooper;
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    I2cDeviceSynchImpl rangeSensor;
    I2cDeviceSynchImpl sideRangeSensor;
    double sideRange;
    //ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    BNO055IMU imu;
    Orientation angles;

    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    double angleZ = 0;

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    double WHEEL_SIZE_IN = 4;
    public int ROTATION = 1220; // # of ticks for 40-1 gear ratio
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
    double GEAR_RATIO = 40;
    double     COUNTS_PER_INCH         = (ROTATION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_SIZE_IN * Math.PI) * (40 / GEAR_RATIO);
    double DIST = 18;
    double SIDE_DIST = 30;
    double backup = -2;
    double overBeacon1 = 2;
    double overBeacon2 = 2;
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init(HardwareMap hardwareMap, Telemetry telem) {

        // Define and Initialize Motors
        leftMotor   = hardwareMap.dcMotor.get("l");
        rightMotor  = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //armMotor.setPower(0);

        // Set all motors to runIMU without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        scooper = hardwareMap.dcMotor.get("scooper");

        state = false;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = telem;
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");
        rangeA = hardwareMap.i2cDevice.get("range sensor");// Primary LEGO Light Sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeA = hardwareMap.i2cDevice.get("r side range");// Primary LEGO Light Sensor
        sideRangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        rangeSensor.engage();
        sideRangeSensor.engage();

        //angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //origAngle = angles.firstAngle;

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        lightSensor.enableLed(true);
    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    int getOpticalDistance(I2cDeviceSynchImpl rangeSensor) {
        return rangeSensor.read(0x04, 2)[1]  & 0xFF;
    }

    int getcmUltrasonic(I2cDeviceSynchImpl rangeSensor){
        return rangeSensor.read(0x04, 2)[0]  & 0xFF;
    }

    void toWhiteLine(boolean wall) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        if (!wall) {
            leftMotor.setPower(APPROACH_SPEED * .4);
            rightMotor.setPower(APPROACH_SPEED * .4);
        }

        while (lightSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        // Stop all motors
        stopRobot();

        if (!wall) {
            encoderDrive(APPROACH_SPEED * .4, 1, 1, 1);
        }
        else
            encoderDrive(APPROACH_SPEED * .4, 2, 2, 2);
    }

    void turn(int turnAngle) {
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = IMUheading();

        double angDiff = turnAngle-angleZ; //positive: turn left
        //if (Math.abs(angDiff) > 180) angDiff = angDiff % 180;

        if (angDiff < 0) { //turns right
            leftMotor.setPower(APPROACH_SPEED * .6 );
            rightMotor.setPower(-APPROACH_SPEED * .6);

            while (angDiff < 0) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;

                if (Math.abs(angDiff) < 90) {
                    leftMotor.setPower(APPROACH_SPEED * .2);
                    rightMotor.setPower(-APPROACH_SPEED * .2);
                }
                else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(APPROACH_SPEED * .05);
                    rightMotor.setPower(-APPROACH_SPEED * .05);
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        else if (angDiff > 0) { //turns left
            leftMotor.setPower(-APPROACH_SPEED);
            rightMotor.setPower(APPROACH_SPEED);

            while (angDiff > 0) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;

                if (Math.abs(angDiff) < 90) {
                    leftMotor.setPower(-APPROACH_SPEED * .2);
                    rightMotor.setPower(APPROACH_SPEED * .2);
                }
                else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(-APPROACH_SPEED * .05);
                    rightMotor.setPower(APPROACH_SPEED * .05);
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void approachBeacon() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Momentarily stop
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(200);

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();

        if (getcmUltrasonic(rangeSensor) > DIST * 2) {
            leftMotor.setPower(APPROACH_SPEED * .8);
            rightMotor.setPower(APPROACH_SPEED * .8);

            while (getcmUltrasonic(rangeSensor) > DIST * 2) {

                telemetry.log().add("Left power" + leftMotor.getPower());
                telemetry.log().add("Right power" + rightMotor.getPower());
                telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
                telemetry.update();

                idle();
            }
            //Momentarily stop
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(100);
        }

        if (getcmUltrasonic(rangeSensor) > DIST) {
            leftMotor.setPower(APPROACH_SPEED * .4);
            rightMotor.setPower(APPROACH_SPEED * .4);
            while (getcmUltrasonic(rangeSensor) > DIST) {

                telemetry.log().add("Left power" + leftMotor.getPower());
                telemetry.log().add("Right power" + rightMotor.getPower());
                telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
                telemetry.update();

                idle();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(100);
        }

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void followLineBlueSide() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(.2);
        rightMotor.setPower(-.2);
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (getcmUltrasonic(rangeSensor) > 9){
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if(lightSensor.getLightDetected() > WHITE_THRESHOLD){
                telemetry.addLine("Moving right");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            } else {
                telemetry.addLine("Moving left");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void followLineRedSide() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(-.2);
        rightMotor.setPower(.2);
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (getcmUltrasonic(rangeSensor) > 9){
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if(lightSensor.getLightDetected() > WHITE_THRESHOLD){
                telemetry.addLine("Moving right");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            } else {
                telemetry.addLine("Moving left");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            }
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void pushBlueButton() {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = leftColorSensor.blue();
        int rightBlue = rightColorSensor.blue();

        do{
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());

            telemetry.update();

            if(leftColorSensor.blue() > rightColorSensor.blue()){// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is blue");
                telemetry.update();

                rightMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                leftMotor.setPower(0);
            } else if(rightColorSensor.blue() > leftColorSensor.blue()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is blue");
                telemetry.update();

                leftMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                rightMotor.setPower(0);
            } else if(leftColorSensor.red() > leftColorSensor.blue() &&
                    rightColorSensor.red() > rightColorSensor.blue()){
                //red button has been pressed
                telemetry.log().add("beacon is red");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                leftMotor.setPower(APPROACH_SPEED);
                rightMotor.setPower(0);

            } else if(getcmUltrasonic(rangeSensor) > 8) {
                encoderDrive(APPROACH_SPEED, 1, 1, 1);
            } else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.log().add("blue is not detected");
                telemetry.update();
                break;
            }
            telemetry.update();
            sleep(1500);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftMotor.setPower(-APPROACH_SPEED * .8);
            rightMotor.setPower(-APPROACH_SPEED * .8);
            sleep(40);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());
            telemetry.update();

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } while (!verifyBlue());

        telemetry.log().add("end of the push button method");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    void pushRedButton() {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = leftColorSensor.blue();
        int rightBlue = rightColorSensor.blue();

        do{
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left red: ", leftColorSensor.red());
            telemetry.addData("Right red: ", rightColorSensor.red());

            telemetry.update();

            if(leftColorSensor.red() > rightColorSensor.red()){// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is red");
                telemetry.update();

                rightMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                leftMotor.setPower(0);
            } else if(rightColorSensor.red() > leftColorSensor.red()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is red");
                telemetry.update();

                leftMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                rightMotor.setPower(0);
            } else if(leftColorSensor.blue() > leftColorSensor.red() && rightColorSensor.blue() > rightColorSensor.red()){
                //red button has been pressed
                telemetry.log().add("beacon is blue");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                leftMotor.setPower(APPROACH_SPEED);
                rightMotor.setPower(0);
            } else if(getcmUltrasonic(rangeSensor) > 8) {
                encoderDrive(APPROACH_SPEED, 1, 1, 1);
            } else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.log().add("red is not detected");
                telemetry.update();
                break;
            }
            telemetry.update();
            sleep(1500);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightMotor.setPower(-APPROACH_SPEED * .8);
            leftMotor.setPower(-APPROACH_SPEED * .8);
            sleep(80);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            telemetry.addData("Left red: ", leftColorSensor.red());
            telemetry.addData("Right red: ", rightColorSensor.red());
            telemetry.update();
        } while  (!verifyRed());

        telemetry.log().add("end of the push button method");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    boolean verifyBlue() {
        if(leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        /*else if (leftColorSensor.red() == rightColorSensor.red()
                && leftColorSensor.blue() == rightColorSensor.blue()
                && leftColorSensor.red() > 2
                && rightColorSensor.red() > 2)
            throw new RuntimeException("Color Sensor problems");*/

        if(leftColorSensor.blue() > leftColorSensor.red() && rightColorSensor.blue() > rightColorSensor.red()){
            telemetry.addLine("Beacon is blue");
            return true;
        }
        /*else if(Math.abs(leftColorSensor.blue() - rightColorSensor.blue()) < 2){
            return true;
        }*/
        telemetry.addLine("Beacon is red");
        return false;
    }

    boolean verifyRed() {
        if(leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        /*else if (leftColorSensor.red() == rightColorSensor.red()
                && leftColorSensor.blue() == rightColorSensor.blue()
                && leftColorSensor.red() > 2
                && rightColorSensor.red() > 2)
            throw new RuntimeException("Color Sensor problems");*/

        if(leftColorSensor.red() > leftColorSensor.blue() && rightColorSensor.red() > rightColorSensor.blue()){
            telemetry.addLine("Beacon is red");
            return true;
        }
        /*else if(Math.abs(leftColorSensor.blue() - rightColorSensor.blue()) < 2){
            return true;
        }*/
        telemetry.addLine("Beacon is blue");
        return false;
    }

    void maintainDist() {

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sideRange = getcmUltrasonic(sideRangeSensor);
        angleZ = IMUheading();
        telemetry.addData("Side Range: ", getcmUltrasonic(sideRangeSensor));
        telemetry.addData("Angle", angleZ);
        telemetry.update();
        double distCorrect = SIDE_DIST - sideRange; //positive if too close

        //makes angle closer to 0
        leftMotor.setPower(APPROACH_SPEED * .6 + angleZ/50 - distCorrect/60);
        rightMotor.setPower(APPROACH_SPEED * .6 - angleZ/50 + distCorrect/60);

    }

    public void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

        leftMotor.setPower(Math.abs(speed));
        rightMotor.setPower(Math.abs(speed));

        while ((runtime.seconds() < timeoutS) &&
                (leftMotor.isBusy() && rightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.addData("Left motor busy", leftMotor.isBusy());
            telemetry.addData("Right motor busy", rightMotor.isBusy());
            telemetry.update();
        }
        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void shoot() {
        EncoderShooter(scaleShooterPower(0.9));//0.6//0.7
        sleep(2000);
        scooper.setPower(1);
        sleep(2500);
        EncoderShooter(0);
        scooper.setPower(0);
    }

    public void EncoderShooter(double speed) {
        shooter1.setPower(speed);
        shooter2.setPower(speed);
    }

    public double scaleShooterPower(double intialPower) {
        double MAX_VOLTAGE=13.7;
        double currentVoltage= hardwareMap.voltageSensor.get("drive").getVoltage();
        double scaledPower=MAX_VOLTAGE*intialPower/currentVoltage;
        telemetry.addData("Scaled power: ", scaledPower);
        return scaledPower;
    }
}