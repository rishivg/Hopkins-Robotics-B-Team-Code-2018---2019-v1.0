package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;

@Autonomous(name="Encodrs")
public class Encodr extends LinearOpMode {

    CyGoat goat = new CyGoat();
    private GoldAlignDetector detector;
    int rotations = 1440;



    @Override
    public void runOpMode() {
        goat.init(hardwareMap);
        goat.imu();

        goat.phone.setPosition(0.137);

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
        telemetry.addData("Help", "Plz Help");

        // Set up detector
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

        detector.enable(); // Start the detector!

        while (!opModeIsActive()) {

            // Gold goes here
        }





        //hook(29792);
        colorDetector();

        //drive(2, 0.3);
        telemetry.update();

    }

    public void colorDetector() {

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
        detector.enable();
        if(detector.getAligned()) {

            telemetry.addLine("Gold Found Center");
            //sleep(1500);
            drive(8030,0.4);

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
        telemetry.update();
    }

    public void drive(int distance, double speed){

        goat.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        goat.front_left.setDirection(DcMotor.Direction.FORWARD);
        goat.back_left.setDirection(DcMotor.Direction.FORWARD);
        goat.front_right.setDirection(DcMotor.Direction.REVERSE);
        goat.back_right.setDirection(DcMotor.Direction.REVERSE);


        goat.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        goat.front_left.setTargetPosition(distance);
        goat.back_left.setTargetPosition(distance);
        goat.front_right.setTargetPosition(distance);
        goat.back_right.setTargetPosition(distance);

        goat.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        while(Math.abs(goat.front_left.getCurrentPosition() - distance) > 50){
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

        goat.hook.setTargetPosition(distance);

        goat.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goat.hook.setPower(1);

        while(Math.abs(goat.hook.getCurrentPosition() - (distance)) > 50){
        }
        goat.hook.setPower(0);

        goat.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}