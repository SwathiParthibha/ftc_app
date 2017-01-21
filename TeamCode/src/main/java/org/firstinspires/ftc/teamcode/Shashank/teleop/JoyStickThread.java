package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by spmeg on 10/22/2016.
 */

public class JoyStickThread extends Thread {
    private Gamepad gamepad = null;
    private ThreadMsger msger = null;
    private volatile boolean requestedStop = false;

    public JoyStickThread(Gamepad gamepadInstance, ThreadMsger msgerInstance){
        gamepad = gamepadInstance;
        msger = msgerInstance;
    }

    @Override
    public void run() {
        msger.setTestValue(true);
        while(!requestedStop){
            requestedStop = msger.isRequestStop();
            msger.setLeftJoystickValue(gamepad.left_stick_y);
            msger.setRightJoystickValue(gamepad.right_stick_y);
        }
    }

    public void requestStop(){
        requestedStop = true;
    }
}
