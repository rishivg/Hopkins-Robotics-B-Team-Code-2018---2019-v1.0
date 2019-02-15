package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoDriveTrain extends LinearOpMode{

    CyGoat goat = new CyGoat();

    public DcMotor rightEncoder;
    public DcMotor leftEncoder;

    public LocationPoint location;
    public double orientation;

    public LocationPoint originalLocation;
    public double originalOrientation;

    private ElapsedTime runtime = new ElapsedTime();

    static final double PPR = 1440;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = PPR * 1 / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_DEGREE = 0;

    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;
    static final double STRAFE_SPEED = 0.5;

    public void runOpMode() {

        goat.init(hardwareMap);
        goat.imu();

        while(!opModeIsActive()){
        }


    }


    void driveTo(LocationPoint target, double power) {//called once, not in loop
        originalLocation = new LocationPoint(location.getX(), location.getY());
        double targetDistance = originalLocation.getDistanceTo(target);
        double targetOrientation = originalLocation.getAngleTo(target);

        turnTo(targetOrientation, power); //turn toward target
        encoderDrive(power * DRIVE_SPEED, targetDistance, 10);//drive toward target
    }

    void turnTo(double target, double power) {
        originalOrientation = orientation;
        int turn_direction = (target - originalOrientation > 0) ? 1 : -1; //may not work
        encoderTurn(turn_direction * power * TURN_SPEED, Math.abs(target - originalOrientation), 10);
    }

    void updateLocation() {
        int avg_encoder = (Math.abs(leftEncoder.getCurrentPosition()) + Math.abs(rightEncoder.getCurrentPosition())) / 2;
        double avg_distance = avg_encoder / COUNTS_PER_INCH;
        location.setX(originalLocation.x + avg_distance * Math.cos(orientation));
        location.setY(originalLocation.y + avg_distance * Math.sin(orientation));
    }

    void updateOrientation() {
        int avg_encoder = (Math.abs(leftEncoder.getCurrentPosition()) + Math.abs(rightEncoder.getCurrentPosition())) / 2;
        double avg_degrees = avg_encoder / COUNTS_PER_DEGREE;
        orientation = originalOrientation + avg_degrees;
    }

    public void encoderDrive(double power, double distance, double timeout) {//distance must be positive; goes forward distance inches
        if (distance < 0) {
            throw new IllegalArgumentException("Distance must be positive");
        }

        int targetEncoderPosition = (int) (distance * COUNTS_PER_INCH);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int absLeftEncoderPos = Math.abs(leftEncoder.getCurrentPosition());
        int absRightEncoderPos = Math.abs(rightEncoder.getCurrentPosition());

        runtime.reset();

        while (absLeftEncoderPos < targetEncoderPosition && absRightEncoderPos < targetEncoderPosition && runtime.seconds() < timeout) {
            if (absLeftEncoderPos - absRightEncoderPos > 0) {
                goat.front_right.setPower(-power * DRIVE_SPEED);
                goat.front_left.setPower(power * DRIVE_SPEED * (1 - .01 * (absLeftEncoderPos - absRightEncoderPos)));
                goat.back_right.setPower(-power * DRIVE_SPEED);
                goat.back_left.setPower(power * DRIVE_SPEED * (1 - .01 * (absLeftEncoderPos - absRightEncoderPos)));
            } else if (absRightEncoderPos - absLeftEncoderPos > 0) {
                goat.front_right.setPower(-power * DRIVE_SPEED * (1 - .01 * (absRightEncoderPos - absLeftEncoderPos)));
                goat.front_left.setPower(power * DRIVE_SPEED);
                goat.back_right.setPower(-power * DRIVE_SPEED * (1 - .01 * (absRightEncoderPos - absLeftEncoderPos)));
                goat.back_left.setPower(power * DRIVE_SPEED);
            } else {
                goat.front_right.setPower(-power * DRIVE_SPEED);
                goat.front_left.setPower(power * DRIVE_SPEED);
                goat.back_right.setPower(-power * DRIVE_SPEED);
                goat.back_left.setPower(power * DRIVE_SPEED);
            }
            absLeftEncoderPos = Math.abs(leftEncoder.getCurrentPosition());
            absRightEncoderPos = Math.abs(rightEncoder.getCurrentPosition());
            updateLocation();
        }
        goat.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        goat.front_left.setPower(0);
        goat.front_right.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
    }

    public void encoderTurn(double power, double degrees, double timeout) { //Negative power for right turn

        int targetEncoderPosition = (int) (degrees * COUNTS_PER_DEGREE);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int absLeftEncoderPos = Math.abs(leftEncoder.getCurrentPosition());
        int absRightEncoderPos = Math.abs(rightEncoder.getCurrentPosition());

        runtime.reset();

        while (absLeftEncoderPos < targetEncoderPosition && absRightEncoderPos < targetEncoderPosition && runtime.seconds() < timeout) {
            if (absLeftEncoderPos - absRightEncoderPos > 0) {
                goat.front_right.setPower(-power * DRIVE_SPEED);
                goat.front_left.setPower(-power * DRIVE_SPEED * (1 - .01 * (absLeftEncoderPos - absRightEncoderPos)));
                goat.back_right.setPower(-power * DRIVE_SPEED);
                goat.back_left.setPower(-power * DRIVE_SPEED * (1 - .01 * (absLeftEncoderPos - absRightEncoderPos)));
            } else if (absRightEncoderPos - absLeftEncoderPos > 0) {
                goat.front_right.setPower(-power * DRIVE_SPEED * (1 - .01 * (absRightEncoderPos - absLeftEncoderPos)));
                goat.front_left.setPower(-power * DRIVE_SPEED);
                goat.back_right.setPower(-power * DRIVE_SPEED * (1 - .01 * (absRightEncoderPos - absLeftEncoderPos)));
                goat.back_left.setPower(-power * DRIVE_SPEED);
            } else {
                goat.front_right.setPower(power * TURN_SPEED);
                goat.front_left.setPower(power * TURN_SPEED);
                goat.back_right.setPower(power * TURN_SPEED);
                goat.back_left.setPower(power * TURN_SPEED);
                //}
                absLeftEncoderPos = Math.abs(leftEncoder.getCurrentPosition());
                absRightEncoderPos = Math.abs(rightEncoder.getCurrentPosition());
                updateOrientation();
            }
        }

        goat.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        goat.front_right.setPower(0);
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
    }
}