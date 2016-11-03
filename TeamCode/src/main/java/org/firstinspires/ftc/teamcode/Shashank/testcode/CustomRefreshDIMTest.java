package org.firstinspires.ftc.teamcode.Shashank.testcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.usb.RobotArmingStateNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by spmeg on 10/29/2016.
 */
@Autonomous(name = "CustomRefreshDIMTest", group = "Tests")
public class CustomRefreshDIMTest extends LinearOpMode {
    private DeviceInterfaceModule module;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("lcs");
        module = hardwareMap.deviceInterfaceModule.get("dim");

        waitForStart();

        RobotLog.d("Am now closing the module");
        telemetry.addData(">", "Am now closing the module");
        module.resetDeviceConfigurationForOpMode();

        while ((opModeIsActive())){
            telemetry.addData(">", "Am now attempting to read data");
            telemetry.addData("color argb", colorSensor.argb());
            telemetry.update();
        }
    }
}
