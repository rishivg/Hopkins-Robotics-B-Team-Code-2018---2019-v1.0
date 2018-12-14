package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Severus", group="Potter")
public class AutoAutonomous extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private GoldAlignDetector detector;

    CyGoat goat = new CyGoat();
    moveFuncs driv = new moveFuncs();

    encoderMap encoder1 = new encoderMap();
    encoderMap encoder2 = new encoderMap();

    double x = 0; //From drop, driving out is +x
    double y = 0; //From drop, driving left is +y, driving right is -y
    int fullRotationCount = 0;
    double previousAngle = 0;
    double angle = 0;

    //@Override
    public void runOpMode(){
        goat.init(hardwareMap);
        goat.imu();
        telemetry.addData(">", "Ready ayy");
        telemetry.update();

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 375; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        while(!opModeIsActive()){
        }
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        // S T A R T /\/\ S T A R T /\/\ S T A R T /\/\ S T A R T \\
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////

        double startTime = runtime.milliseconds();

        //driv.detach();
        //detector.disable();
        goat.phone.setPosition(0.6);
        detector.enable();
        sleep(1500);
        goat.deadBois.setPosition(0.3);

        sleep(1500);

        encoder1.init(goat.encoder1.getVoltage());
        encoder2.init(goat.encoder2.getVoltage());

        if(detector.getAligned() == true) {

            telemetry.addLine("Gold Found Center");

            goat.phone.setPosition(0);
            sleep(1500);
            driveTo(0,14.75,0,.5);

        } else {
            detector.disable();
            goat.phone.setPosition(0.45);
            detector.enable();
            sleep(1500);


        }

        if(detector.getAligned() == true) {
            detector.disable();
            telemetry.addLine("Gold Found Left");
            goat.phone.setPosition(0);
            sleep(1500);
            driveTo(-9,10.75,0,.5);


        } else {
            detector.disable();
            telemetry.addLine("Gold found Right");
            goat.phone.setPosition(0);
            driveTo(9,10.75,0,.5);


        }

        goat.phone.setPosition(0);
        sleep(1500);
        driveTo(0,4,0,0.5);
        //driveTo(-20,5,0,.5);

    }

    public void driveTo(double desiredX, double desiredY, double desiredAngle, double threshold){
        double distAway = 0;
        double errorX = 0;
        double errorY = 0;
        do{
            updateCoordinates();
            errorX = x-desiredX;
            errorY = desiredY-y;
            distAway = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2));
            double theta = Math.atan2(errorY, errorX);
            double errorAngle = angle-desiredAngle;
            theta = -(theta-Math.PI);
            telemetry.addData("ErrorX", errorX);
            telemetry.addData("ErrorY", errorY);
            autoDrive(theta, 0.2*distAway, 0); //0.1*errorAngle
        }
        while(distAway > threshold && opModeIsActive());
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);
    }

    public void autoDrive(double theta, double magnitude, double Protate){

        double modifiedTheta = theta + Math.PI/4 - angle;

        magnitude *= (1-Math.abs(Protate)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        telemetry.addData("MagDrive", magnitude);
        telemetry.update();
        double Px = -magnitude * -Math.cos(modifiedTheta);
        double Py = -magnitude * -Math.sin(modifiedTheta);

        goat.front_left.setPower(Py + Protate);
        goat.back_left.setPower(-1 * (Px + Protate));
        goat.back_right.setPower(Py - Protate);
        goat.front_right.setPower(-1 * (Px - Protate));
    }


    public void updateCoordinates(){
        encoder1.update(goat.encoder1.getVoltage());
        encoder2.update(goat.encoder2.getVoltage());
        angleOverflow();
        angle = getHeading();
        double deltaYAngle = encoder2.deltaAngle;
        double deltaXAngle = -encoder1.deltaAngle;
        double movementAngle = Math.atan2(deltaYAngle, deltaXAngle);
        x -= Math.sqrt(Math.pow(deltaXAngle, 2)+Math.pow(deltaYAngle, 2))*Math.cos(movementAngle-angle);
        y += Math.sqrt(Math.pow(deltaXAngle, 2)+Math.pow(deltaYAngle, 2))*Math.sin(movementAngle-angle);
    }
    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0
        double heading = getHeading() - fullRotationCount*(2*Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if(heading < Math.PI/2 && previousAngle > 3*Math.PI/2){
            fullRotationCount++;
        }
        if(heading > 3*Math.PI/2 && previousAngle < Math.PI/2){
            fullRotationCount--;
        }
        previousAngle = heading;
    }
    public double getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
        Orientation angles = goat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        heading = (Math.PI/180)*heading;
        if(heading < 0){
            heading = (2*Math.PI) + heading;
        }
        heading += fullRotationCount*(2*Math.PI);
        return heading;
    }
}

