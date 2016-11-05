package org.firstinspires.ftc.teamcode.Pranav;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a our robot.
 * Our files for usage examples.
 */

/*

This is our Class where all our functions are included that are used in the entire project
Please do not edit any of the functions as it will affect other programs in the project

If you have accomplished something please tell us so we can implement into the class.
*/
public class MecanumHardware extends LinearOpMode
{
    /* Public OpMode members. */

    //Currently there are two motors defined. As the season progresses we may add additional motors
    DcMotor frontRight = null;
    DcMotor backRight = null;
    DcMotor frontLeft = null;
    DcMotor backLeft = null;

    //Where all Sensors are defined
    public ModernRoboticsI2cGyro gyro = null;
    // public LightSensor legoLineSensor = null;
    // public ModernRoboticsI2cRangeSensor rangeSensor = null;

    //1000 Milliseconds
    public int SECOND = 1000;

    //Motor Variables
    public int ROTATION = 1220; // # of ticks
    public int MOTOR_POWER = 1;

    //Z-Axis of the Modern Robotics Gyro Sensor
    int heading = 0;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor *///Empty Constructor
    public MecanumHardware()
    {

    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
        {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    //This function sets the motors to 0 stopping the Robot
    public void stopRobot()
    {
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public void stopAndResetEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //A basic go straight function that uses encoders to track its distance
    public void drive(int distance, double speed) throws InterruptedException
    {
        telemetry.addData("Starting to Drive", frontRight.getCurrentPosition() / ROTATION);
        telemetry.update();

        runUsingEncoder();

        stopAndResetEncoder();

        frontRight.setPower(speed);
        backRight.setPower(speed);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);

        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);

        runToPosition();

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() /*&& opModeIsActive()*/ )
        {

        }

        stopRobot();

        runUsingEncoder();

        telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
        telemetry.update();
    }


    //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
    public void turnGyro(String direction, int angle, double speed) throws InterruptedException
    {
        int motorDirectionChange = 0;

        runWithoutEncoder();

        if (direction.equals("left"))
        {
            motorDirectionChange = -1;
        }
        else
        if (direction.equals("right"))
        {
            motorDirectionChange = 1;
        }

        while ((heading > angle + 5 || heading < angle - 2))
        {
            frontRight.setPower(MOTOR_POWER * speed * motorDirectionChange);
            backRight.setPower(MOTOR_POWER * speed * motorDirectionChange);

            frontLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);
            backLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);

            heading = gyro.getHeading();

            telemetry.addData("We Are Turning", heading);
            telemetry.addData("Gyro Value", gyro.getHeading());
            telemetry.update();
        }

        stopRobot();

        telemetry.addData("We Are Done Turning", heading);
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontRight = hwMap.dcMotor.get("motor_2");
        backRight = hwMap.dcMotor.get("motor_4");
        frontLeft = hwMap.dcMotor.get("motor_3");
        backLeft = hwMap.dcMotor.get("motor_1");

        //Define and Initialize Sensors
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        // legoLineSensor = hwMap.lightSensor.get("legoLineSensor");
        // rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //Configure the Direction of the Motors
        frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        stopRobot();

        // Set all motors to run using encoders.
        // May want to use RUN_WITHOUT_ENCODER if encoders are not installed.
        runUsingEncoder();

        //Calibrate the Modern Robotics Gyro Sensor
        gyro.calibrate();

        //Turn on the LED of the Lego Line Sensor
        //legoLineSensor.enableLed(true);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}

