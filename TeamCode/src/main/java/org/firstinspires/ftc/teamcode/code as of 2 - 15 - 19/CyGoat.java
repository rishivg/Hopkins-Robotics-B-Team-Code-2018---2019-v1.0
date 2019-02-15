package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CyGoat {

    //DRIVE//
    public DcMotor front_left   = null;
    public DcMotor front_right  = null;
    public DcMotor back_left    = null;
    public DcMotor back_right   = null;

    //public DcMotor linSlide1 = null;
    //public DcMotor linSlide2 = null;

    //public DcMotor arm_motor_1 = null;
    //public DcMotor arm_motor_2 = null;

    public DcMotor hook = null;

    //public DcMotor intake1 = null;
    //public DcMotor intake2 = null;
    //public DcMotor intake3 = null;
    //public DcMotor intake4 = null;

    public Servo phone = null;
    public Servo marker = null;



    public AnalogInput arm_encoder = null;

    public AnalogInput potentiometer = null;


    BNO055IMU imu;

    HardwareMap hwMap = null;

    /* Constructor */
    public CyGoat(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        //DRIVE//
        front_left   = hwMap.dcMotor.get("front_left");
        front_right  = hwMap.dcMotor.get("front_right");
        back_left    = hwMap.dcMotor.get("back_left");
        back_right   = hwMap.dcMotor.get("back_right");

        hook         = hwMap.dcMotor.get("hook");

        //intake1      = hwMap.servo.get("intake1");
        //intake2      = hwMap.servo.get("intake2");
        //intake3      = hwMap.servo.get("intake3");
        //intake4      = hwMap.servo.get("intake4");

        phone        = hwMap.servo.get("phone");
        marker       = hwMap.servo.get("marker");

        //linSlide1    = hwMap.dcMotor.get("lin_slide1");
        //linSlide2    = hwMap.dcMotor.get("lin_slide2");

        //arm_motor_1 = hwMap.dcMotor.get("arm1");
        //arm_motor_2 = hwMap.dcMotor.get("arm2");


        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

       /* front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    */
        hook.setDirection(DcMotor.Direction.FORWARD);

        //linSlide1.setDirection(DcMotor.Direction.FORWARD);
        //linSlide1.setDirection(DcMotor.Direction.REVERSE);

        //arm_motor_1.setDirection(DcMotor.Direction.FORWARD);
        //arm_motor_2.setDirection(DcMotor.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoder1 = hwMap.analogInput.get("encoder1");
        //encoder2 = hwMap.analogInput.get("encoder2");

        imu = hwMap.get(BNO055IMU.class, "imu");
    }
    public void imu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}