package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="SHIT")
public class SHIT extends LinearOpMode {

    CyGoat goat = new CyGoat();


    private GoldAlignDetector detector;
    int rotations = 1440;


    @Override
    public void runOpMode() {
        goat.init(hardwareMap);
        goat.imu();

        goat.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
        telemetry.addData("Help", "Plz Help");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 500; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
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


        goat.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        goat.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        goat.front_left.setDirection(DcMotor.Direction.FORWARD);
        goat.back_left.setDirection(DcMotor.Direction.FORWARD);
        goat.front_right.setDirection(DcMotor.Direction.REVERSE);
        goat.back_right.setDirection(DcMotor.Direction.REVERSE);

        hook(35120);

        sleep(4000);


        detector.enable();
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
        goat.phone.setPosition(0.137);
        goat.marker.setPosition(0.32);

        sleep(1500);
        if (detector.getAligned()) {
            telemetry.addLine("Gold Found Center");
            sleep(1500);
            drive(2750, 0.7);
            //goat.marker.setPosition(.650);
            sleep(3000);


        } else {
            detector.disable();
            goat.phone.setPosition(0.328);

            detector.enable();
            sleep(1000);

            if (detector.getAligned()) {
                telemetry.addLine("Gold Found Right");
                rotRight(490, 0.3);
                sleep(200);
                drive(1900, 0.4);
                rotLeft(997, 0.3);
                drive(950, 0.7);
                //goat.marker.setPosition(0.650);
                sleep(500);


            } else {
                detector.disable();
                telemetry.addLine("Gold found Left");
                rotLeft(518, 0.3);
                sleep(200);
                drive(1900, 0.4);
                rotRight(997, 0.3);
                drive(900, 0.7);
                //goat.marker.setPosition(0.650);
                sleep(500);

            }


        }


        telemetry.update();

    }

    public void drive(int distance, double speed) {
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

        while (Math.abs(goat.front_left.getCurrentPosition() - distance) > 50) {
        }

        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);


    }

    public void rotRight(int distance, double speed) {
        goat.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        goat.front_left.setTargetPosition(distance);
        goat.back_left.setTargetPosition(distance);
        goat.front_right.setTargetPosition(-distance);
        goat.back_right.setTargetPosition(-distance);

        goat.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

        while (Math.abs(goat.front_left.getCurrentPosition() - distance) > 50) {
        }

        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);


    }

    public void rotLeft(int distance, double speed) {
        goat.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        goat.front_left.setTargetPosition(-distance);
        goat.back_left.setTargetPosition(-distance);
        goat.front_right.setTargetPosition(distance);
        goat.back_right.setTargetPosition(distance);

        goat.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goat.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        while (Math.abs(goat.front_right.getCurrentPosition() - distance) > 50) {
        }

        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);


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