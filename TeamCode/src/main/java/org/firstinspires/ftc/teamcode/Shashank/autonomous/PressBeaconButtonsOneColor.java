package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.vuforia.Obb2D;

/**
 * Created by spmeg on 10/28/2016.
 */
@Autonomous(name = "PressBeaconButtonsOneColor", group = "Auto")
public class PressBeaconButtonsOneColor extends OpMode{
    private ColorSensor colorSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void init() {
        rangeSensor = this.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        colorSensor  = hardwareMap.colorSensor.get("rcs");

        leftMotor = hardwareMap.dcMotor.get("l");
        rightMotor = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DbgLog.msg("Finished INIT");
    }

    @Override
    public void loop() {

        int leftRed = colorSensor.red();
        int leftBlue = colorSensor.blue();

        double savedTime = this.time;
        if(leftRed > leftBlue && !verify()){
            //write the code here to press the left button
            leftMotor.setPower(0.3);
            rightMotor.setPower(0.0);
            DbgLog.msg("red is greater than blue");
            telemetry.addData("", "red is greater than blue");
        } else if(leftBlue > leftRed  && !verify()){
            //write the code here to press the right button
            rightMotor.setPower(0.3);
            leftMotor.setPower(0.0);
            DbgLog.msg("blue is greater than red");
            telemetry.addData("", "blue is greater than red");
        } else{
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        telemetry.addData("power of left motor", leftMotor.getPower());
        telemetry.addData("power of right motor", rightMotor.getPower());
        telemetry.addData("cm in ultrasonic", rangeSensor.cmUltrasonic());
        telemetry.addData("cm in optical", rangeSensor.cmOptical());
        telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue()));
        this.updateTelemetry(telemetry);

    }

    private void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean verify(){
        if(colorSensor.red() > colorSensor.blue()){
            return true;
        }
        return false;
    }

}

