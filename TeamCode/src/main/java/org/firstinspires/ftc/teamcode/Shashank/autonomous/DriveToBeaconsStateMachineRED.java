package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mrinali.HardwarePushbot;

/**
 * Created by spmeg on 10/22/2016.
 */
@Autonomous(name = "DriveToBeaconsStateMachineRED", group = "Autonomous")
public class DriveToBeaconsStateMachineRED extends OpMode {
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    // OpticalDistanceSensor

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.Mrinali.HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    //lightSensor;   // Alternative MR ODS sensor
    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.4;
    double DIST = 11;
    ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;


    @Override
    public void init() {
        rangeSensor = this.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        leftColorSensor  = hardwareMap.colorSensor.get("rcs");

        rightColorSensor = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        rightColorSensor.setI2cAddress(i2cAddr);

        leftMotor = hardwareMap.dcMotor.get("l");
        rightMotor = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        while (gyro.isCalibrating())  {
            sleep(50);
        }

        robot.init(hardwareMap);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");                // Primary LEGO Light Sensor
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
    }

    @Override
    public void loop() {
        heading = gyro.getHeading();
        angleZ  = gyro.getIntegratedZValue();

        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("cm in ultrasonic", rangeSensor.cmUltrasonic());
        telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", leftColorSensor.alpha(), leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue()));
        telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", rightColorSensor.alpha(), rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue()));
        telemetry.addData("verify", verify());
        telemetry.addData("Light Level", lightSensor.getLightDetected());
        telemetry.update();

        //go to the white line
        try {
            toWhiteLine();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        //overshoot the line and use the light sensor again to
        //turn right and find the line again
        int targetAngleZ =  angleZ + 32;
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD){
            leftMotor.setPower(-0.1);
            rightMotor.setPower(-0.3);
        }
        sleep(750);
        robot.leftMotor.setPower(0);
        approachBeacon();
        pushButton();

        /*telemetry.update();
        // Turn right
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(750);
        robot.leftMotor.setPower(0);

        telemetry.update();
        approachBeacon();
        pushButton();

        telemetry.update();
        // Go backwards slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);

        telemetry.update();
        // Turn left - parallel to wall
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(APPROACH_SPEED);
        sleep(750); //REPLACE: Use gyro

        telemetry.update();
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);
        sleep(750);

        telemetry.update();
        try {
            toWhiteLine();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        telemetry.update();
        //Turn right - to beacon
        robot.rightMotor.setPower(APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(750); //REPLACE: Use gyro

        telemetry.update();
        approachBeacon();
        pushButton();

        telemetry.update();*/
    }

    void toWhiteLine() throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
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

        while (rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }

        //Momentarily stop
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sleep(200);

        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .25);
            robot.rightMotor.setPower(APPROACH_SPEED * .25);
        }

        while (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void pushButton() {
        // Pushes button, then straightens
        // REPLACE: Code to push button, use color sensor

        int leftRed = leftColorSensor.red();

        int rightRed = rightColorSensor.red();

        if(leftRed > rightRed && !verify()){
            //write the code here to press the left button
            leftMotor.setPower(0.3);
            rightMotor.setPower(0.0);

            //wait three seconds
            verify();
        } else if(rightRed > leftRed && !verify()){
            //write the code here to press the right button
            rightMotor.setPower(0.3);
            leftMotor.setPower(0.0);
            verify();
        } else{
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    private boolean verify() {
        if(leftColorSensor.argb() == 0 || rightColorSensor.argb() == 0)
            return false;

        if(leftColorSensor.argb() == 255 || rightColorSensor.argb() == 255)
            return false;

        if(Math.abs(leftColorSensor.red() - rightColorSensor.red()) < 2){
            return true;
        }

        return false;
    }

    private void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
