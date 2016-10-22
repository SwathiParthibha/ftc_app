package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by spmeg on 10/8/2016.
 */
@Autonomous(name = "CustomColorSensorTest", group = "Tests")
public class CustomColorSensorTest extends OpMode {
    private ColorSensor colorSensor = null;
    private DeviceInterfaceModule deviceInterfaceModule = null;

    @Override
    public void init() {
        colorSensor = this.hardwareMap.colorSensor.get("lcs");
        deviceInterfaceModule = this.hardwareMap.deviceInterfaceModule.get("dim");
        colorSensor.enableLed(true);
    }

    @Override
    public void loop() {

        telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue()));
        telemetry.update();
    }
}
