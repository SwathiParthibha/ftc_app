package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "EncoderTeleop", group = "Teleop")
public class EncoderTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;

    private boolean state;
    boolean swap=false;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");

        state = false;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        if(swap==true)
        {
            double temp=left;
            left=right;
            right=temp;
        }

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if(gamepad1.dpad_down){
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swap=false;
        } else if(gamepad1.dpad_up){
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            swap=true;
        }

        if(gamepad1.dpad_right){
            sweeper.setPower(0.7);
            scooper.setPower(1);
        }

        if(gamepad1.left_trigger > 0){
            scooper.setPower(1);
        } else if(gamepad1.left_bumper){
            scooper.setPower(-0.7);
        } else {
            scooper.setPower(0);
        }

        if(gamepad1.a){
            shooter1.setPower(0.9);
            shooter2.setPower(0.9);
        } else if(gamepad1.b) {
            shooter1.setPower(0.6);
            shooter2.setPower(0.6);
        }
        else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }


        if(gamepad1.right_bumper){
            sweeper.setPower(0.7);
        } else if(gamepad1.right_trigger > 0){
            sweeper.setPower(-0.7);
        } else {
            sweeper.setPower(0);
        }

        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.update();
    }

    public void EncoderShooter()
    {
        int left = shooter1.getCurrentPosition();
        int right = shooter2.getCurrentPosition();

    }
}
