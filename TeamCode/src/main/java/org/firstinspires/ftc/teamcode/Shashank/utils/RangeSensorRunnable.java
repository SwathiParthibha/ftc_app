package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by spmega on 1/5/17.
 */

public class RangeSensorRunnable implements Runnable {
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private boolean requestStop = false;

    public RangeSensorRunnable(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public RangeSensorRunnable(Telemetry telemetry, ElapsedTime runtime) {
        this.telemetry = telemetry;
        this.runtime = runtime;
    }

    public void run() {
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while (!requestStop) {
            telemetry.log().add("This is printing from a thread: " + runtime.toString());
            telemetry.update();
        }
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    public void requestStop(){
        requestStop = true;
    }
}
