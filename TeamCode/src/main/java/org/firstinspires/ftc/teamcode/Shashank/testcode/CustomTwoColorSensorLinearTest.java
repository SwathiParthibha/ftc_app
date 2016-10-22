package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by spmega4567 on 10/7/16.
 */
@Autonomous(name = "CustomTwoColorSensorLinearTest", group = "Tests")
public class CustomTwoColorSensorLinearTest extends LinearOpMode
{
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        waitForStart();

        while (opModeIsActive())
        {
            int left = leftColorSensor.argb();
            int right = rightColorSensor.argb();
            telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", leftColorSensor.alpha(), leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue()));
            telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", rightColorSensor.alpha(), rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue()));
            this.updateTelemetry(telemetry);

            Thread.sleep(500);
        }

    }
}
