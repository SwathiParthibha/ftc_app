package org.firstinspires.ftc.teamcode.Shashank.utils;

/**
 * Created by spmeg on 11/5/2016.
 */

public class StateMachine {
    private static String name = "";
    private Thread thread = null;

    public StateMachine(String name, Runnable callBack) {
        thread = new Thread(callBack);
    }

    public StateMachine(String name, Thread thread) {
        this.thread = thread;
    }
}
