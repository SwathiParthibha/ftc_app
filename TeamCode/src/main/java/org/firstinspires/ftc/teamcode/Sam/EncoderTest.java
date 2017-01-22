package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;


@TeleOp(name = "Encoder Test", group = "Teleop")
public class EncoderTest extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;

    private double RequestedRPM=4000;
    private double power=0;
    private long dt=100;
    private double previous_position1=0;
    private double current_position1=0;
    private double current_rpm1=0;
    private double previous_rpm1=0;
    private double error1=0;
    private double previous_error1=0;
    private double integral1=0;
    private double derivative1=0;
    private double adjustment1=0;
    private double previous_position2=0;
    private double current_position2=0;
    private double current_rpm2=0;
    private double previous_rpm2=0;
    private double error2=0;
    private double previous_error2=0;
    private double integral2=0;
    private double derivative2=0;
    private double adjustment2=0;

    private boolean startrunnning=false;
    private boolean running=false;
    private String output="";


    private boolean ShooterPowerCont=true;

    final Runnable ShooterPower = new Runnable() {
        public void run() {

            while (ShooterPowerCont) {
                synchronized (this) {
                    try {

                        current_position1=shooter1.getCurrentPosition();

                        current_rpm1 = (current_position1 - previous_position1) / (int) dt;

                        error1=RequestedRPM-current_rpm1;
                        adjustment1=error1 ;
                        previous_position1=current_position1;
                        previous_rpm1=current_rpm1;

                        output="output: "+String.format("%.4f",(power+adjustment1));
                        output+="adjust: "+String.format("%.4f",(adjustment1));
                        output+="curr"+current_rpm1;
                        output+="Time: "+getRuntime();
                        //output+="power: "+power;


                        current_position2=shooter2.getCurrentPosition();

                        current_rpm2 = (current_position2 - previous_position2) / (int) dt;

                        adjustment2=(RequestedRPM-current_rpm2);
                        previous_position2=current_position2;
                        previous_rpm2=current_rpm2;

                        //output+="error2: "+error2;
                        //output+="adjust: "+adjustment2;
                        //output+="curr"+current_rpm2;


                        if(startrunnning)
                        {
                            startrunnning=false;
                            running=true;
                            shooter1.setPower(power);
                           // shooter2.setPower(power);
                        }

                        if(running)
                        {
                            shooter1.setPower(power+adjustment1);
                            //shooter2.setPower(power-adjustment2);
                            //shooter2.setPower(power);
                        }
                        else
                        {
                            integral1=0;
                            previous_rpm1=0;
                            previous_error1=0;
                            previous_position1=0;
                            integral2=0;
                            previous_rpm2=0;
                            previous_error2=0;
                            previous_position2=0;
                            shooter1.setPower(0);
                            shooter2.setPower(0);
                            //shooter2.setPower(power);
                        }



                        sleep(dt);
                    } catch (Exception e) {
                    }
                }
            }
        }
    };
    final Thread Shooter = new Thread(ShooterPower);


    private boolean state;
    boolean swap=false;



    @Override
    public void init() {

        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");

        state = false;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Shooter.start();

    }

    @Override
    public void loop() {

        int shooting1= shooter1.getCurrentPosition();
        int shooting2= shooter2.getCurrentPosition();

        if(gamepad2.a){
            EncoderShooter(scaleShooterPower(0.55));//0.7//0.9
        } else if(gamepad2.b) {
            //EncoderShooter(scaleShooterPower(0.8));//0.6//0.7
            power=0.55;
            startrunnning=true;
        }
        else if(gamepad2.y)
        {
            //EncoderShooter(0.2);
        }
        else {
            EncoderShooter(0);
            power=0;
            running=false;
        }

  telemetry.addData("shooting1", shooting1);
        telemetry.addData("shooting2", shooting2);
        telemetry.addData("Out",output);
        telemetry.update();
    }

    @Override
    public void stop() {
        ShooterPowerCont=false;
        super.stop();
    }


    public double prevTime=0;

    public double requiredPWR1=0.8;
    public double requiredPWR2=0.8;

    public void EncoderShooter(double speed)
    {
        if(speed!=0) {

            double Kp = 0.001;
            double Ki = 0.00001;
            double Kd = 0.00001;


            if (getRuntime() - prevTime > 0.25) {//only update every 10ms
                current_position1 = shooter1.getCurrentPosition();
                current_position2 = shooter2.getCurrentPosition();
                current_rpm1 = (current_position1 - previous_position1) / (getRuntime() - prevTime);
                current_rpm2 = (current_position2 - previous_position2) / (getRuntime() - prevTime);


                if(current_rpm1<RequestedRPM)
                {//we need to speed up
                    requiredPWR1+=Kp;
                }
                else if(current_rpm1>RequestedRPM)
                {//we need to slow down
                    requiredPWR1-=Kp;
                }
                if(current_rpm2<RequestedRPM)
                {//we need to speed up
                    requiredPWR2+=Kp;
                }
                else if(current_rpm2>RequestedRPM)
                {//we need to slow down
                    requiredPWR2-=Kp;
                }


                previous_position1 = current_position1;
                previous_rpm1 = current_rpm1;
                previous_position2 = current_position2;
                previous_rpm2 = current_rpm2;
                prevTime = getRuntime();
            }


            telemetry.addData("requiredPWR1: ", String.format("%.4f", requiredPWR1));
            telemetry.addData("requiredPWR2: ", String.format("%.4f", requiredPWR2));
            telemetry.addData("curr1", current_rpm1);
            telemetry.addData("curr2", current_rpm2);
            telemetry.addData("Time: ", "" + getRuntime());



            shooter1.setPower(requiredPWR1);
            shooter2.setPower(requiredPWR2);
            //shooter2.setPower(speed);

        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

    }

    public double scaleShooterPower(double intialPower)
    {
        double MAX_VOLTAGE=13.7;

        double currentVoltage= hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();

        double scaledPower=MAX_VOLTAGE*intialPower/currentVoltage;

        telemetry.addData("Scaled power: ", scaledPower);

        return (scaledPower*0+intialPower);



    }


}



