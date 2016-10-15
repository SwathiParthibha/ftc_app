package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a our robot.
 * Our files for usage examples.
 */
public class RobotHardware
{
    int SECOND = 1000;

    int ROTATION = 1220;
    int MOTOR_POWER = 1;

    double black = 0.20;
    double white = 0.48;
    double avg = (black + white)/ 2;

    int heading = 0;

    /* Public OpMode members. */
    public DcMotor left = null;
    public DcMotor right = null;
    public GyroSensor gyro = null;
    public LightSensor light = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor *///Empty Constructor
    public RobotHardware()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        left = hwMap.dcMotor.get("motorLeft");
        right = hwMap.dcMotor.get("motorRight");
        gyro = hwMap.gyroSensor.get("gyro");
        light = hwMap.lightSensor.get("light");


        gyro.calibrate();

        light.enableLed(true);

        left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        left.setPower(0);
        right.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    /*    while (gyro.isCalibrating()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                //do nothing
            }


        }
*/

    }
    public void waitForTick(long periodMs)
    {
        /***
         * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
         * periodic tick.  This is used to compensate for varying processing times for each cycle.
         * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
         *
         * @param periodMs Length of wait cycle in mSec.
         */
        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
        {
            try
            {
                Thread.sleep(remaining);
            } catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    public void stopRobot()
    {
        left.setPower(0);
        right.setPower(0);
    }
    public void goStraight(double speed, long time) throws InterruptedException
    {
        left.setPower(speed);
        right.setPower(speed);


        Thread.sleep(time);
    }

    public void Drive(int Distance, double Speed) throws InterruptedException
    {

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setTargetPosition(Distance);
        left.setTargetPosition(Distance);

        // Turn On RUN_TO_POSITION
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right.setTargetPosition(10);
        left.setTargetPosition(10);

        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setPower(Speed);
        left.setPower(Speed);

        while (left.isBusy())
        {

        }

        stopRobot();
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Turn(String Direction, int angle, double Speed) throws InterruptedException {
        int MotorDirectionChange = 0;

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Direction.equals("left"))
        {
            MotorDirectionChange = -1;
        } else if (Direction.equals("right"))
        {
            MotorDirectionChange = 1;
        }

        while ((heading > angle + 5 || heading < angle - 2))
        {
            right.setPower(Speed * MotorDirectionChange);
            left.setPower(-1 * Speed * MotorDirectionChange);

            heading = gyro.getHeading();
        }
        stopRobot();
    }

    public void goToLine(double speed) throws InterruptedException
    {
        left.setPower(speed);
        right.setPower(speed);


        while(light.getLightDetected() < avg)
        {
            //do nothing
        }
        stopRobot();
    }




}

