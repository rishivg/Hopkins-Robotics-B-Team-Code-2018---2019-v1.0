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


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Home Depot", group="Linear Opmode")

public class HomeDepot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    CyGoat goat = new CyGoat();

    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {

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

        detector.enable();

        // run until the end of the match (driver presses STOP)
        while (!opModeIsActive()) {}

        if(detector.getAligned() == true) {

            telemetry.addLine("Gold Found Center");
            sleep(1000);

        } else {
            detector.disable();
            goat.phone.setPosition(0.37);
            detector.enable();
            sleep(1500);


        }

        if(detector.getAligned() == true) {
            detector.disable();
            telemetry.addLine("Gold Found Left");
            //goat.phone.setPosition(0);
            sleep(1000);


        } else {
            detector.disable();
            telemetry.addLine("Gold found Right");
            sleep(1000);
            //driveTo(9,10.75,0,.5);


        }

        landing(0.25, 2000, 2000, 0.4, 1000);
        fowardShift(0.2,1000);
        rightShift(0.4, 1750);
        fowardShift(0.2, 2800);
        markerGoal(1000);


    }

    public void landing(double armPower, long armTime, long sleepTime, double movePower, long moveTime)
    {
        delatch(armPower, armTime);
        sleep(sleepTime);
        leftShift(movePower, moveTime);
    }

    public void delatch(double power, long time) {

        goat.hook.setPower(power);

        sleep(time);
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

    private void markerDefault(long time) {
        //goat.marker.setPosition(0);
        //sleep(time);
    }

    private void markerGoal(long time) {
        //goat.marker.setPosition(1);
        //sleep(time);
        //goat.marker.setPosition(0);
        //sleep(time);
    }


}

