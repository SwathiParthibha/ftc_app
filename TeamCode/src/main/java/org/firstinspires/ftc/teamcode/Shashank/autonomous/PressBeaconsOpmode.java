package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by spmeg on 10/21/2016.
 */
@Autonomous(name = "PressBeaconsOpmode", group = "AutonomousTests")
public class PressBeaconsOpmode extends OpMode {
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ModernRoboticsI2cRangeSensor rangeSensor;

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
    }

    @Override
    public void loop() {

        int leftRed = leftColorSensor.red();
        int leftBlue = leftColorSensor.blue();
        int leftGreen = leftColorSensor.green();

        int rightRed = rightColorSensor.red();
        int rightBlue = rightColorSensor.blue();
        int rightGreen = rightColorSensor.green();

        double currentTime = this.time;
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

        telemetry.addData("power of left motor", leftMotor.getPower());
        telemetry.addData("power of right motor", rightMotor.getPower());
        telemetry.addData("cm in ultrasonic", rangeSensor.cmUltrasonic());
        telemetry.addData("cm in optical", rangeSensor.cmOptical());
        telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", leftColorSensor.alpha(), leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue()));
        telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", rightColorSensor.alpha(), rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue()));
        telemetry.addData("verify", verify());
        this.updateTelemetry(telemetry);

    }

    private void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
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

    private void moveBack() {
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        double currentTime = this.time;
        //wait 2 seconds
        sleep(2000);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
