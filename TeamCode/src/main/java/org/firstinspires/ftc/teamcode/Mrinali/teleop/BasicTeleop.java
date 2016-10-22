package org.firstinspires.ftc.teamcode.Mrinali.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by spmeg on 10/22/2016.
 */

public class BasicTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        leftMotor = this.hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        leftMotor.setPower(left);
        rightMotor.setPower(right);

        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.update();
    }
}
