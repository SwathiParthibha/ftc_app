package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by spmeg on 10/22/2016.
 */

public class MotorThread extends Thread{
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private ThreadMsger msger = null;

    private volatile boolean requestedStop = false;

    public MotorThread(ThreadMsger msger, DcMotor leftMotor, DcMotor rightMotor){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    @Override
    public void run() {
        while(!requestedStop){
            leftMotor.setPower(-msger.getLeftJoystickValue());
            rightMotor.setPower(-msger.getRightJoystickValue());
        }
    }

    public void requestStop(){
        requestedStop = true;
    }
}
