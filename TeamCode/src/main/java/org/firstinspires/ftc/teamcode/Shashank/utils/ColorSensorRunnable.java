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
    private String RED_KEY = "red";
    private String BLUE_KEY = "blue";
    private String GREEN_KEY = "green";
    private String ARGB_KEY = "argb";
    private String ALPHA_KEY = "alpha";

    public ColorSensorRunnable(Telemetry telemetry, ElapsedTime runtime, ThreadSharedObject threadSharedObject, ColorSensor colorSensor) {
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.colorSensor = colorSensor;
        RED_KEY = "colorSensor"+colorSensor.getDeviceName()+"red";
        BLUE_KEY = "colorSensor"+colorSensor.getDeviceName()+"blue";
        GREEN_KEY = "colorSensor"+colorSensor.getDeviceName()+"green";
        ARGB_KEY = "colorSensor"+colorSensor.getDeviceName()+"argb";
        ALPHA_KEY = "colorSensor"+colorSensor.getDeviceName()+"alpha";
    }

    public ColorSensorRunnable(Telemetry telemetry, ElapsedTime runtime, ColorSensor colorSensor, double timeout) {
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.colorSensor = colorSensor;
        this.timeout = timeout;
    }

    public String getRED_KEY() {
        return RED_KEY;
    }

    public String getBLUE_KEY() {
        return BLUE_KEY;
    }

    public String getGREEN_KEY() {
        return GREEN_KEY;
    }

    public String getARGB_KEY() {
        return ARGB_KEY;
    }

    public String getALPHA_KEY() {
        return ALPHA_KEY;
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
            threadSharedObject.setInteger(getARGB_KEY(), colorSensor.argb());
            threadSharedObject.setInteger(getRED_KEY(), colorSensor.red());
            threadSharedObject.setInteger(getBLUE_KEY(), colorSensor.blue());
            threadSharedObject.setInteger(getGREEN_KEY(), colorSensor.green());
            threadSharedObject.setInteger(getALPHA_KEY(), colorSensor.alpha());
        }
        //if you are not running a loop, make sure to periodically check if the requestStop is still false
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    //call this method when you want to stop the thread
    public void requestStop(){
        requestStop = true;
    }
}
