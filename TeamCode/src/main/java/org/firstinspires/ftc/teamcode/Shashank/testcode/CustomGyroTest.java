package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by spmeg on 10/22/2016.
 */
@Autonomous(name = "CustomGyroTest", group = "Tests")
public class CustomGyroTest extends OpMode {
    ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    int angleZ = 0;

    @Override
    public void init() {
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        while (gyro.isCalibrating())  {
            sleep(50);
        }
    }

    @Override
    public void loop() {
        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.update();
    }

    private void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
