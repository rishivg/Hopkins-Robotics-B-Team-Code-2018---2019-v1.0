package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Crater Hater", group="Linear Opmode")

public class CraterHater extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    CyGoat goat = new CyGoat();

    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        goat.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        goat.front_left.setDirection(DcMotor.Direction.FORWARD);
        goat.back_left.setDirection(DcMotor.Direction.FORWARD);
        goat.front_right.setDirection(DcMotor.Direction.REVERSE);
        goat.back_right.setDirection(DcMotor.Direction.REVERSE);

        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 175; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        goat.init(hardwareMap);
        goat.imu();

        goat.phone.setPosition(0.137);
        detector.enable();

        while (!opModeIsActive()) {}

        sleep(1000);
        //colorDetector();

    }

    public void colorDetector() {

        if(detector.getAligned() == true) {

            telemetry.addLine("Gold Found Center");
            sleep(1500);

        } else {
            detector.disable();
            goat.phone.setPosition(0.287);
            detector.enable();
            sleep(1500);

        }

        if(detector.getAligned() == true) {
            telemetry.addLine("Gold Found Right");
            sleep(1500);


        } else {
            detector.disable();
            telemetry.addLine("Gold found Left");
            sleep(1500);

        }
    }

    public void drive(int distance, double speed){


        goat.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        goat.front_left.setTargetPosition(distance * 1440);
        goat.back_left.setTargetPosition(distance * 1440);
        goat.front_right.setTargetPosition(distance * 1440);
        goat.back_right.setTargetPosition(distance * 1440);

        goat.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        while(Math.abs(goat.front_left.getCurrentPosition() - (distance * 1440)) > 50){
        }

        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);

        goat.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void hook(int distance){
        goat.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        goat.hook.setTargetPosition(distance * 1440);

        goat.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goat.hook.setPower(1);

        while(Math.abs(goat.hook.getCurrentPosition() - (distance * 1440)) > 50){
        }
        goat.hook.setPower(0);

    }

    private void leftShift(double speed, long time) {

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

        sleep(time);
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);
    }

    private void rightShift(double speed, long time) {

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        sleep(time);
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);
    }

    private void fowardShift(double speed, long time) {

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(speed);

        sleep(time);
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);
    }
}
