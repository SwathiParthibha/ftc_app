package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shashank.testcode.TestMoveState;
import org.firstinspires.ftc.teamcode.Shashank.testcode.TestState;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.statemachine.States;

/**
 * Created by spmeg on 1/20/2017.
 */

public class AutoStates extends States {
    private static Telemetry telemetry = null;

    public AutoStates(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static State telemetry(StateName stateName, final StateName nextStateName){
        telemetry.log().add("Now returning state from method telemetry");
        return new TestState(nextStateName, 2);
    }

    public static State drive(StateName stateName, final StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor){
        return new TestMoveState(stateName, nextStateName, 1, leftMotor, rightMotor);
    }

    public static State lineFollow(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, I2cDeviceSynchImpl rangeSensor){
        return new LineFollowState(telemetry, stateName, nextStateName, leftMotor, rightMotor, lightSensor,  rangeSensor);
    }

}
