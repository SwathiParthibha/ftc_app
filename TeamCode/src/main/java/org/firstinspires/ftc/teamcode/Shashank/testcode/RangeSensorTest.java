package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by spmeg on 10/21/2016.
 */
@TeleOp(name = "RangeSensorTest", group = "Tests")
public class RangeSensorTest extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void init() {
        rangeSensor = this.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        leftMotor = hardwareMap.dcMotor.get("l");
        rightMotor = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //all this opmode does is stay 3 cm from the wall and when gamepad.x is pressed, it backs up about 3cm
        boolean isPressed = gamepad1.x;

        if(isPressed){
            moveback();
        } else if(rangeSensor.cmUltrasonic() > 3){
            //maintain the position
            while(rangeSensor.cmUltrasonic() > 3){
                leftMotor.setPower(0.2);
                rightMotor.setPower(0.2);
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        telemetry.addData("Range sensor cmUltrasonic", rangeSensor.cmUltrasonic());
        telemetry.update();
    }

    private void moveback() {
        double currentPos = rangeSensor.cmUltrasonic();
        while(rangeSensor.cmUltrasonic() < (currentPos + 3.0)){
            leftMotor.setPower(-0.3);
            rightMotor.setPower(-0.3);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
