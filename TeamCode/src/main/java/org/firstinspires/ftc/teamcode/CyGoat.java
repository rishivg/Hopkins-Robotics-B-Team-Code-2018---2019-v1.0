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

    //public ColorSensor colorSensor = null;
    
    //DRIVE//
    public DcMotor front_left   = null;
    public DcMotor front_right  = null;
    public DcMotor back_left    = null;
    public DcMotor back_right   = null;
    
    public DcMotor Intake1 = null;
    public DcMotor Intake2 = null;
    
    public DcMotor arm_motor = null;
    
    public AnalogInput potentiometer = null;
    
    
    BNO055IMU imu;

    HardwareMap hwMap = null;

    /* Constructor */
    public CyGoat(){
        
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        
        /* RETRIEVING STUFF FROM PHONES */
        
        //DRIVE//
        front_left   = hwMap.dcMotor.get("front_left");
        front_right  = hwMap.dcMotor.get("front_right");
        back_left    = hwMap.dcMotor.get("back_left");
        back_right   = hwMap.dcMotor.get("back_right");
        
        Intake1 = hwMap.dcMotor.get("intake_left");
        Intake2 = hwMap.dcMotor.get("intake_right");
        
        arm_motor = hwMap.dcMotor.get("arm");
        
        //potentiometer = hardwareMap.analogInput.get("potent");
        
        //colorSensor        = hwMap.dcMotor.get("color_sensor")
        
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
        Intake1.setDirection(DcMotor.Direction.FORWARD);
        Intake2.setDirection(DcMotor.Direction.FORWARD);
        Intake1.setDirection(DcMotor.Direction.REVERSE);
        Intake2.setDirection(DcMotor.Direction.REVERSE);
        
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");
        
        // Add sum more stuff//
    }
    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}
