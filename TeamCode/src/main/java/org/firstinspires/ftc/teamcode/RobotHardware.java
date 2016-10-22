package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    //Where all Sensors are defined
    public ModernRoboticsI2cGyro modernRoboticsGyroSensor = null;
    public LightSensor legoLineSensor = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    public BNO055IMU IMU = null;
    public ColorSensor leftColorSensor = null;
    public ColorSensor rightColorSensor = null;

    //1000 Milliseconds
    public int SECOND = 1000;

    //Ultra Sonic Distance
    int ultraSonicDistance = 11;

    //Motor Variables
    public int ROTATION = 1220; // # of ticks
    public int MOTOR_POWER = 1;

    //Lego Line Sensor Thresholds
    double WHITE_THRESHOLD = 0.3;

    //Z-Axis of the Modern Robotics Gyro Sensor
    int heading = 0;

    //Color Sensor Variables
    int leftColorSensorRed = leftColorSensor.red();

    int rightColorSensorRed = rightColorSensor.red();

    //IMU Variables
    public Orientation angles;
    public Acceleration gravity;
    int imuHeading = 0;

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
        modernRoboticsGyroSensor = hwMap.get(ModernRoboticsI2cGyro.class, "modernRoboticsGyroSensor");
        legoLineSensor = hwMap.lightSensor.get("legoLineSensor");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        leftColorSensor  = hwMap.colorSensor.get("leftColorSensor");
        rightColorSensor = hwMap.colorSensor.get("rightColorSensor");

        //IMU Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        IMU = hwMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);

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
        modernRoboticsGyroSensor.calibrate();

        //Turn on the LED of the Lego Line Sensor
        legoLineSensor.enableLed(true);

        /*This prevents the Modern Robotics Gyro Sensor from
          incorrectly calibrating before the start of Autonomous
        */

        while (modernRoboticsGyroSensor.isCalibrating())
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
    public void drive(int Distance, double Speed) throws InterruptedException
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

    //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
    public void turnGyro(String Direction, int angle, double Speed) throws InterruptedException
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
        motorRight.setPower(MOTOR_POWER * Speed * MotorDirectionChange);
        motorLeft.setPower(-MOTOR_POWER * Speed * MotorDirectionChange);

        heading = modernRoboticsGyroSensor.getHeading();
    }
    stopRobot();
    }

    /*
    //A basic Turn function that uses the Adafruit IMU Sensor to calculate the angle
    public void turnIMU(String Direction, int angle, double Speed) throws InterruptedException
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

        while ((imuHeading > angle + 5 || imuHeading < angle - 2))
        {
            motorRight.setPower(MOTOR_POWER * Speed * MotorDirectionChange);
            motorLeft.setPower(-MOTOR_POWER * Speed * MotorDirectionChange);

            imuHeading = IMU.getAngularOrientation();
        }
        stopRobot();
    }
    */

    //A basic Line Following function that uses the Lego Line Sensor
    public void goToLine(double speed) throws InterruptedException
    {
        motorLeft.setPower(speed);
        motorRight.setPower(speed);

        while(legoLineSensor.getLightDetected() < WHITE_THRESHOLD)
        {
            //do nothing
        }

        stopRobot();
    }

    public void approachBeacon(double Speed)
    {
        // Drive to set distance away, slow down, stop at set distance
        if (rangeSensor.getDistance(DistanceUnit.CM) > ultraSonicDistance * 3)
        {
            motorRight.setPower(Speed);
            motorLeft.setPower(Speed);
        }

        stopRobot();

        if (rangeSensor.getDistance(DistanceUnit.CM) > ultraSonicDistance)
        {
            motorRight.setPower(Speed * 0.25);
            motorLeft.setPower(Speed * 0.25);
        }

        stopRobot();
    }

    public void pushButtonRed()
    {
        if(leftColorSensorRed > rightColorSensorRed )
        {
            //write the code here to press the left button
            motorLeft.setPower(MOTOR_POWER * 0.3);
            motorRight.setPower(MOTOR_POWER * 0);

        }
        else if(rightColorSensorRed > leftColorSensorRed )
        {
            //write the code here to press the right button
            motorRight.setPower(MOTOR_POWER * 0.3);
            motorLeft.setPower(MOTOR_POWER * 0.0);

        }
        else
        {
            stopRobot();
        }
    }
}

