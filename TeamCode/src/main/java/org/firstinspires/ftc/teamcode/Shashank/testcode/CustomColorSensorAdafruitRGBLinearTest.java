package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by spmega4567 on 10/7/16.
 */
@Autonomous(name = "CustomColorSensorAdafruitRGBLinearTest", group = "Tests")
@Disabled
public class CustomColorSensorAdafruitRGBLinearTest extends LinearOpMode
{
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftColorSensor  = hardwareMap.colorSensor.get("lcs");

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        waitForStart();

        while (opModeIsActive())
        {
            int left = leftColorSensor.argb();
            int right = rightColorSensor.argb();
            telemetry.addData("left alpha", String.format("a=%d", leftColorSensor.alpha()));
            telemetry.addData("left red", String.format("r=%d", leftColorSensor.red()));
            telemetry.addData("left gree", String.format("g=%d", leftColorSensor.green()));
            telemetry.addData("left blue", String.format("b=%d", leftColorSensor.blue()));
            telemetry.addData("right alpha", String.format("a=%d", rightColorSensor.alpha()));
            telemetry.addData("right red", String.format("r=%d", rightColorSensor.red()));
            telemetry.addData("right green", String.format("g=%d", rightColorSensor.green()));
            telemetry.addData("right blue", String.format("b=%d", rightColorSensor.blue()));
            this.updateTelemetry(telemetry);

            Thread.sleep(500);
        }

    }
}
