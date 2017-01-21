package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by spmeg on 10/8/2016.
 */
@Autonomous(name = "CustomTouchSensorTest", group = "Tests")
@Disabled
public class CustomTouchSensorTest extends LinearOpMode {

    private TouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.touchSensor.get("touch");

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Touch Sensor:",touchSensor.isPressed());
            this.updateTelemetry(telemetry);

            Thread.sleep(500);
        }

    }
}
