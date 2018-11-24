
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.CyGoat;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous
public class AutoTest2 extends OpMode {

    CyGoat goat = new CyGoat();

    double currentX;
    double currentY;



    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private SamplingOrderDetector detector;

    //Vuforia variables
    private OpenGLMatrix lastLocation;
    private boolean targetVisible;
    Dogeforia vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Detector object
    GoldAlignDetector detector2;

    //@Override
    public void init() {
        // Setup camera and Vuforia parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Set Vuforia parameters
        parameters.vuforiaLicenseKey = "AabYYhL/////AAABmQqiQRvPcEUFjO6HrSiHAtJhA4XsLR02RYWNdMS1pZANLIsI4e64du9DhT2VVbbRAzjC5GaPwRHPFD0kIJ18G7hns7kxjqYsbupQqwpdN1kdLEE5kNwZCB7Om7ASU4Yb3TDqK4WfN1IHcVocvX3M6X1HfKOWR8ktu0956ctgxTgWT1BNmirQI2M5PUtmFNXBPF6WnW5R/ju6N57uNHOeFy6kHzhazkh0c76QL352HZr9b66r5q4o8NSSILrCeEgT2syQYk9QXVl0VZDh7OqlkX3QnxAcGiQiNfLbI7KqAlGiPkYdDQXdfcfixtgx9MWxk3ZTiYOxwAlZNYZRHwR53ylW7IsNyipvvqdRj4NFWSW5";
        parameters.fillCameraMonitorViewParent = true;

        // Init Dogeforia
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Set target names
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        // Set trackables' location on field
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix.translation(0, -mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        //Set camera displacement
        final int CAMERA_FORWARD_DISPLACEMENT = -8;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 178;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 222;     // eg: Camera is ON the robot's center line

        // Set phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        //Set info for the trackables
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //Activate targets
        targetsRoverRuckus.activate();


        detector2 = new GoldAlignDetector(); // Create a gold aligndetector
        detector2.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);

        detector2.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector2.useDefaults(); // Use default settings
        detector2.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector2.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        //Setup Vuforia
        vuforia.setDogeCVDetector(detector2); // Set the Vuforia detector
        vuforia.enableDogeCV(); //Enable the DogeCV-Vuforia combo
        vuforia.showDebug(); // Show debug info
        vuforia.start(); // Start the detector

        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");


        // Setup detector
        detector = new SamplingOrderDetector(); // Create the detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set detector to use default settings

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable(); // Start detector
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    //@Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    //@Override
    public void start() {
        //Reset timer
        runtime.reset();

        //set up CyGoat initiailization and BNO55 IMU
        goat.init(hardwareMap);
        goat.imu();

        //detach();

        while (targetVisible = false) {

            turnClock(0.1,30000);

        }
        autoDrive(36, 36, 1, "RIGHT");

        /*
        while (detector.getCurrentOrder().toString() == "UNKNOWN") {
            //autoDrive(0, 0, 0, "BALLS");
            turnClock(0.1,30000);
        }

        if (detector.getCurrentOrder().toString() == "CENTER") {
            while (targetVisible = false) {

                turnClock(0.1,30000);

            }
            autoDrive(36, 36, 1, "RIGHT");
            //autoDrive(24, 24, 1, "RIGHT");
            //autoDrive(0, 60, 1, "TOP");
            //autoDrive(60,60, 1, "LEFT");
            //autoDrive(-24, 60, 1, "LEFT");

        }

        if (detector.getCurrentOrder().toString() == "LEFT") {
            while (targetVisible = false) {

                turnClock(0.1,30000);

            }
            autoDrive(24, 48, 1, "TOP");
            //autoDrive(24,24, 1, "TOP");
            //autoDrive(0,60, 1, "TOP");
            //autoDrive(60,60, 1, "LEFT");
            //autoDrive(-24, 60, 1, "LEFT");
        }

       if (detector.getCurrentOrder().toString() == "RIGHT") {
            while (targetVisible = false) {

                turnClock(0.1,30000);

            }
            autoDrive(48, 24, 1, "RIGHT");
            //autoDrive(24,24, 1, "RIGHT");
            //autoDrive(0, 60, 1, "TOP");
            //autoDrive(60,60, 1, "LEFT");
            //autoDrive(-24, 60, 1, "LEFT");
        }

        */

    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    //@Override
    public void loop() {
        //Assume we can't find a target
        targetVisible = false;

        //Loop through trackables - if we find one, get the location
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                //We found a target! Print data to telemetry
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // Express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            currentX = translation.get(0);
            currentY = translation.get(1);

            // Express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            //No visible target
            telemetry.addData("Visible Target", "none");
        }
        // Update telemetry
        telemetry.update();


        telemetry.addData("Current Order", detector.getCurrentOrder().toString()); // The current result for the frame
        telemetry.addData("Last Order", detector.getLastOrder().toString()); // The last known result
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        vuforia.stop();

        detector.disable();

    }

    public void autoDrive(double desiredX, double desiredY, float speed, String lookat) {

        double Protate = 0;
        double stick_x = 0; //Accounts for Protate when limiting magnitude to be less than 1
        double theta = 0;

        while (Math.abs(currentX - desiredX) < 3 && Math.abs(currentY - desiredY) < 3) {
            VectorF translation = lastLocation.getTranslation();
            double currentX = translation.get(0) / mmPerInch;
            double currentY = translation.get(1) / mmPerInch;
            double gyroAngle = Math.atan2(desiredY - currentY, desiredX - currentX);

            if (lookat == "BALLS") {
                Protate = Math.atan2(36 - currentY, 36 - currentX);
            } else if (lookat == "RIGHT") {
                Protate = Math.atan2(0 - currentY, 72 - currentX);
            } else if (lookat == "TOP") {
                Protate = Math.atan2(72 - currentY, 0 - currentX);
            } else if (lookat == "LEFT") {
                Protate = Math.atan2(0 - currentY, -72 - currentX);
            } else if (lookat == "BOTTOM") {
                Protate = Math.atan2(-72 - currentY, 0 - currentX);
            } else {
                Protate = 0;
            }

            double stick_y = speed * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);

            theta = Math.atan2(stick_y, stick_x) + Math.PI / 4 - gyroAngle;
            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2));
            double Px = magnitude * Math.cos(theta);
            double Py = magnitude * Math.sin(theta);

            telemetry.addData("Magnitude", magnitude);
            telemetry.addData("Front Left", Py - Protate);
            telemetry.addData("Back Left", Px - Protate);
            telemetry.addData("Back Right", Py + Protate);
            telemetry.addData("Front Right", Px + Protate);

            goat.front_left.setPower(Py - Protate);
            goat.back_left.setPower(Px - Protate);
            goat.back_right.setPower(Py + Protate);
            goat.front_right.setPower(Px + Protate);
        }

        // STOP AFTER REACHING DES.LOCATION
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);
    }

    public void driveForward(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        sleep(time);

    }

    public void driveBackward(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

        sleep(time);

    }

    public void leftShift(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(speed);

        sleep(time);

    }

    public void rightShift(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.front_left.setPower(speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(-speed);

        sleep(time);

    }

    public void turnClock(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

        sleep(time);

    }

    public void turnCountClock(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        sleep(time);

    }

    public void RosettaStone(double refSpeed, double refAngle, double refRotationSpeed) {

        double speed = refSpeed;
        double angle = refAngle;
        double rotationSpeed = refRotationSpeed;


        goat.front_left.setPower((speed * Math.sin(-angle + (Math.PI/4)) - rotationSpeed )/ 2);
        goat.front_right.setPower((speed * Math.cos(-angle + (Math.PI/4)) + rotationSpeed )/ 2);
        goat.back_left.setPower((speed * Math.cos(-angle + (Math.PI/4)) - rotationSpeed )/ 2);
        goat.back_right.setPower((speed * Math.sin(-angle + (Math.PI/4)) + rotationSpeed )/ 2);

    }

    public void detach() {

        moveArm(-0.7, 500);
        rightShift(0.3, 250);
        driveForward(0.5, 100);
        leftShift(0.3, 250);

    }

    public void moveArm(double refSpeed, long refTime) {

        double speed = refSpeed;
        long time = refTime;

        goat.arm_motor.setPower(speed);
        sleep(time);
    }

    public void detachToPoint(double refSpeed, double refTime, double refAngle) {



    }

    public static void sleep(long refSleepTime)
    {
        long sleepTime = refSleepTime;

        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }

}
