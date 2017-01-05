package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by spmega on 1/5/17.
 */

public class TemplateRunnableOpMode implements Runnable {
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private boolean requestStop = false;

    public TemplateRunnableOpMode(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public TemplateRunnableOpMode(Telemetry telemetry, ElapsedTime runtime) {
        this.telemetry = telemetry;
        this.runtime = runtime;
    }

    @Override
    public void run() {
        //if you need to run a loop, make sure that you check that requestStop is still false
        while (!requestStop) {
            //simply printing out and updating the telemetry log here
            telemetry.addData("Thread", "This is printing from a thread: " + runtime.toString());
            telemetry.update();
        }
        //if you are not running a loop, make sure to periodically check if the requestStop is still false
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    //call this method when you want to stop the thread
    public void requestStop(){
        requestStop = true;
    }
}
