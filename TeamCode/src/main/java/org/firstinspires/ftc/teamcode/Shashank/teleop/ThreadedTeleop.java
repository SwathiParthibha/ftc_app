package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "ThreadedTeleop", group = "Tests")
public class ThreadedTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private ThreadMsger threadMsger = new ThreadMsger();
    private JoyStickThread joyStickThread = null;
    private MotorThread motorThread = null;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.log().add("Finished init of motors");

        telemetry.log().add("Init motor thread");
        motorThread = new MotorThread(threadMsger, leftMotor, rightMotor);
        telemetry.log().add("Starting motor thread");
        //motorThread.start();
        telemetry.log().add("Started motor thread");

        telemetry.log().add("Init joystick thread");
        joyStickThread = new JoyStickThread(this.gamepad1, threadMsger);
        telemetry.log().add("Starting joystick thread");
        //joyStickThread.start();
        telemetry.log().add("Started joystick thread");
    }

    @Override
    public void loop() {
        double left = -threadMsger.getLeftJoystickValue();
        double right = -threadMsger.getRightJoystickValue();

        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.addData("test Value, true if successful", threadMsger.getTestValue());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        threadMsger.setRequestStop(true);
        joyStickThread.requestStop();
        motorThread.requestStop();
    }
}
