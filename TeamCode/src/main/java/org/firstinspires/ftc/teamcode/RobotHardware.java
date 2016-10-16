package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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
public class RobotHardware
{
    /* Public OpMode members. */

    //Currently there are two motors defined. As the season progresses we may add additional motors
    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;

    //Where all Sensor are defined
    //public GyroSensor ModernRoboticsGyroSensor = null;
    //public LightSensor LegoLineSensor = null;
    public BNO055IMU IMUSensor = null;

    //1000 Milliseconds
    public int SECOND = 1000;

    //Motor Variables
    public int ROTATION = 1220; // # of ticks
    public int MOTOR_POWER = 1;

    //Lego Line Sensor Thresholds
    double black = 0.20;
    double white = 0.48;
    double avg = (black + white)/ 2;

    //Z-Axis of the Modern Robotics Gyro Sensor
    int heading = 0;

    //IMU Variables
    public Orientation angles;
    public Acceleration gravity;

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
        motorLeft = hwMap.dcMotor.get("motorLeft");
        motorRight = hwMap.dcMotor.get("motorRight");

        //Define and Initialize Sensors
        //ModernRoboticsGyroSensor = hwMap.gyroSensor.get("ModernRoboticsGyro");
        //LegoLineSensor = hwMap.lightSensor.get("LegoLineSensor");

        //Define and Initialize the parameters of the IMUSensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Configure the Direction of the Motors
        motorLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Calibrate the Modern Robotics Gyro Sensor
        //ModernRoboticsGyroSensor.calibrate();

        //Turn on the LED of the Lego Line Sensor
        //LegoLineSensor.enableLed(true);

        /*This prevents the Modern Robotics Gyro Sensor from
          incorrectly calibrating before the start of Autonomous
        */
        /*
        while (ModernRoboticsGyroSensor.isCalibrating())
        {
            try
            {
                Thread.sleep(50);
            }

            catch (InterruptedException e)
            {
                //do nothing
            }
        }
    */
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

    //This function sets the motors to 0 stopping the Robot
    public void stopRobot()
    {
        motorLeft.setPower(MOTOR_POWER * 0);
        motorRight.setPower(MOTOR_POWER * 0);
    }

    //A basic go straight function that stops after a certain time
    public void goStraight(double Speed, long Time) throws InterruptedException
    {
        motorLeft.setPower(MOTOR_POWER * Speed);
        motorRight.setPower(MOTOR_POWER * Speed);

        Thread.sleep(Time);
    }

    //A basic go straight function that uses encoders to track its distance
    public void Drive(int Distance, double Speed) throws InterruptedException
    {
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRight.setPower(MOTOR_POWER * Speed);
        motorLeft.setPower(MOTOR_POWER * Speed);

        motorRight.setTargetPosition(Distance);
        motorLeft.setTargetPosition(Distance);

        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorRight.isBusy() && motorLeft.isBusy())
        {

        }
        stopRobot();
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
    //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
    public void Turn(String Direction, int angle, double Speed) throws InterruptedException
    {
        int MotorDirectionChange = 0;

        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Direction.equals("Left"))
        {
            MotorDirectionChange = -1;
        } else if (Direction.equals("Right"))
        {
            MotorDirectionChange = 1;
        }

        while ((heading > angle + 5 || heading < angle - 2))
        {
            motorRight.setPower(Speed * MotorDirectionChange);
            motorLeft.setPower(-1 * Speed * MotorDirectionChange);

            heading = ModernRoboticsGyroSensor.getHeading();
        }
        stopRobot();
    }
    */

    /*
    //A basic Line Following function that uses the Lego Line Sensor
    public void goToLine(double speed) throws InterruptedException
    {
        motorLeft.setPower(speed);
        motorRight.setPower(speed);

        while(LegoLineSensor.getLightDetected() < avg)
        {
            //do nothing
        }

        stopRobot();
    }
    */
}

