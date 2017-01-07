package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by spmeg on 10/15/2016.
 */
@Autonomous(name = "CustomThreeColorSensorLinearTest", group = "Tests")
@Disabled
public class CustomThreeColorSensorTest extends LinearOpMode {
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    ColorSensor thirdColorSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftColorSensor  = hardwareMap.colorSensor.get("lcs");

        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        rightColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("cs");
        I2cAddr i2cAddr2 = I2cAddr.create8bit(0x5c);
        rightColorSensor.setI2cAddress(i2cAddr2);

        waitForStart();

        while (opModeIsActive())
        {
            int left = leftColorSensor.argb();
            int right = rightColorSensor.argb();
            telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", leftColorSensor.alpha(), leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue()));
            telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", rightColorSensor.alpha(), rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue()));
            telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", thirdColorSensor.alpha(), thirdColorSensor.red(), thirdColorSensor.green(), thirdColorSensor.blue()));
            this.updateTelemetry(telemetry);
            Thread.sleep(500);
        }

    }
}