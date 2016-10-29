package org.firstinspires.ftc.teamcode.Shashank.testcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by spmeg on 10/29/2016.
 */

public class CustomRefreshDIMTest extends OpMode {
    private DeviceInterfaceModule module;

    @Override
    public void init() {
        module = this.hardwareMap.deviceInterfaceModule.get("dim");
    }

    @Override
    public void loop() {
    }
}
