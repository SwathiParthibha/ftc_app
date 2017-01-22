package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */

public class PressBeaconState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor = null;

    private StateName stateName = null;
    private StateName nextStateName = null;

    private ColorSensor leftColorSensor, rightColorSensor;

    private Telemetry telemetry = null;

    private boolean hasInitialized = false;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private int timeout = 0;

    private BeaconColor beaconColor = BeaconColor.RED;

    public PressBeaconState(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, ColorSensor leftColorSensor, ColorSensor rightColorSensor, Telemetry telemetry, int timeout, BeaconColor color) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.telemetry = telemetry;
        this.timeout = timeout;
        this.beaconColor = beaconColor;
    }

    @Override
    public void init() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public StateName act() {
        if(isDone())
            return nextStateName;

        if(beaconColor == BeaconColor.RED){
            //if red do this
            if(leftColorSensor.red() > rightColorSensor.red()){
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
        } else if(beaconColor == BeaconColor.BLUE){
            //if blue do this
            if(leftColorSensor.blue() > rightColorSensor.blue()){
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
        } else {
            return nextStateName;
        }

        return stateName;
    }

    @Override
    public boolean isDone() {
        return elapsedTime.seconds() > timeout || isSameColor();
    }

    private boolean isSameColor() {
        if(beaconColor == BeaconColor.RED){
            if(leftColorSensor.red() > leftColorSensor.blue() && rightColorSensor.red() > rightColorSensor.blue())
                return true;
            else
                return false;
        } else if(beaconColor == BeaconColor.BLUE){
            if(leftColorSensor.red() < leftColorSensor.blue() && rightColorSensor.red() < rightColorSensor.blue())
                return true;
            else
                return false;
        } else {
            return false;
        }
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
