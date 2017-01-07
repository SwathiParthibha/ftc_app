package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by spmega on 1/5/17.
 */

public class ColorSensorRunnable implements Runnable {
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private boolean requestStop = false;
    private double timeout = 0;
    private ColorSensor colorSensor = null;
    private ThreadSharedObject threadSharedObject = null;

    public ColorSensorRunnable(Telemetry telemetry, ElapsedTime runtime, ThreadSharedObject threadSharedObject, ColorSensor colorSensor) {
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.colorSensor = colorSensor;
    }

    public ColorSensorRunnable(Telemetry telemetry, ElapsedTime runtime, ColorSensor colorSensor, double timeout) {
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.colorSensor = colorSensor;
        this.timeout = timeout;
    }

    @Override
    public void run() {
        //if you need to run a loop, make sure that you check that requestStop is still false
        while (!requestStop) {
            //simply printing out and updating the telemetry log here
            if(timeout > 0){
                if(runtime.seconds() > timeout)
                    break;
            }

            telemetry.addData("Thread", "This is printing from a thread: " + runtime.toString());
            telemetry.update();
            threadSharedObject.setInteger("colorSensor"+colorSensor.getDeviceName()+"argb", colorSensor.argb());
            threadSharedObject.setInteger("colorSensor"+colorSensor.getDeviceName()+"red", colorSensor.red());
            threadSharedObject.setInteger("colorSensor"+colorSensor.getDeviceName()+"blue", colorSensor.blue());
            threadSharedObject.setInteger("colorSensor"+colorSensor.getDeviceName()+"green", colorSensor.green());
            threadSharedObject.setInteger("colorSensor"+colorSensor.getDeviceName()+"alpha", colorSensor.alpha());
        }
        //if you are not running a loop, make sure to periodically check if the requestStop is still false
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    //call this method when you want to stop the thread
    public void requestStop(){
        requestStop = true;
    }
}
