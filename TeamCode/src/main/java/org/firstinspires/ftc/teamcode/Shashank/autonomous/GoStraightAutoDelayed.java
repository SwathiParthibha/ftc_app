package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by spmeg on 12/18/2016.
 */
@Autonomous(name = "GoStraightAutoDelayed", group = "Auto")
public class GoStraightAutoDelayed extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor   = hardwareMap.dcMotor.get("l");
        rightMotor  = hardwareMap.dcMotor.get("r");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        sleep(10000);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        sleep(1700);

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        while ((opModeIsActive())){
            sleep(500);
            telemetry.update();
        }
    }
}
