package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by spmeg on 10/29/2016.
 */

public class CustomAdafuitMNColorSensorsTest extends OpMode{
    private ColorSensor mnColorSensor;
    private ColorSensor adFruitColorSensor;

    @Override
    public void init() {
        mnColorSensor = this.hardwareMap.colorSensor.get("rcs");
        adFruitColorSensor = this.hardwareMap.colorSensor.get("sensor_color");
    }

    @Override
    public void loop() {

        telemetry.addData("AD Clear", adFruitColorSensor.alpha());
        telemetry.addData("AD Red  ", adFruitColorSensor.red());
        telemetry.addData("AD Green", adFruitColorSensor.green());
        telemetry.addData("AD Blue ", adFruitColorSensor.blue());

        telemetry.addData("AD Clear", mnColorSensor.alpha());
        telemetry.addData("AD Red  ", mnColorSensor.red());
        telemetry.addData("AD Green", mnColorSensor.green());
        telemetry.addData("AD Blue ", mnColorSensor.blue());

    }
}
