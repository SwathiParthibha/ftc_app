package org.firstinspires.ftc.teamcode.Shashank.teleop;

/**
 * Created by spmeg on 10/22/2016.
 */

public class ThreadMsger {
    private double leftJoystickValue = 0;
    private double rightJoystickValue = 0;
    private volatile boolean requestStop = false;

    private boolean testValue = false;

    public synchronized void setLeftJoystickValue(double leftJoystickValue) {
        this.leftJoystickValue = leftJoystickValue;
    }

    public synchronized void setRightJoystickValue(double rightJoystickValue) {
        this.rightJoystickValue = rightJoystickValue;
    }

    public double getLeftJoystickValue() {
        return leftJoystickValue;
    }

    public double getRightJoystickValue() {
        return rightJoystickValue;
    }

    public synchronized void setTestValue(boolean testValue) {
        this.testValue = testValue;
    }

    public boolean getTestValue(){
        return testValue;
    }

    public synchronized boolean isRequestStop() {
        return requestStop;
    }

    public synchronized void setRequestStop(boolean requestStop) {
        this.requestStop = requestStop;
    }
}
