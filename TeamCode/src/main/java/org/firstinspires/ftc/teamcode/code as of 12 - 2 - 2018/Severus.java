/*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Serverus Snape", group="goat autos")
public class Severus extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private CyGoat goat =  new CyGoat();
    moveFuncs driv = new moveFuncs();

    private encoderMap encoder1 = new encoderMap();
    private encoderMap encoder2 = new encoderMap();
    private String blocko = "center";

    private double x = 0; //From drop, driving out is +x
    private double y = 0; //From drop, driving left is +y, driving right is -y
    private int fullRotationCount = 0;
    private double previousAngle = 0;
    private double angle = 0;

    //PID STUFF\\
    private double kRotate[] = {0, 0, 0}; //PID constants for rotation
    private double kDrive[] = {.21, 6.82, 3.72}; //PID constants for linear drive power

    private double eDrive = 0;
    private double eRotate = 0;

    private double iDrive = 0;
    private double iRotate = 0;

    private double dDrive = 0;
    private double dRotate = 0;

    private double lastTime = 0;



    @Override
    public void runOpMode(){

        telemetry.addData(">", "Wait");
        telemetry.update();

        goat.init(hardwareMap);
        telemetry.addData(">", "Loading up");
        telemetry.update();

        goat.imu();
        telemetry.addData(">", "Ready");
        telemetry.update();

        //while(!opModeIsActive()){ }

        //////////////
        ////START////
        ////////////

        double startTime = runtime.milliseconds();
        goat.deadWheels.setPosition(1.0);
        encoder1.init(goat.encoder1.getVoltage());
        encoder2.init(goat.encoder2.getVoltage());

        if(blocko.equals("center")) {

            doPID(-14.4, -0.38, 0, 0.5, 0);
            doPID(-7.2, -1.09, 0, 0.5, 0);
        }
        doPID(-5.07, -23.25, 0, 0.5, 0);
        doPID(7.8, -34.5, 0, 0.5, 0);
        doPID(0, 0, 0, 0.3, 0);

    }
    private void doPID(double desX, double desY, double desAngle, double driveMargin, double rotateMargin){

        updateCoordinates();

        //boolean running = true;
        double driveTime = runtime.milliseconds();
        double errX = x-desX;
        double errY = desY-y;
        double refDrive = eDrive;
        double refRotate = eRotate;

        eDrive = Math.sqrt(Math.pow(errX, 2) + Math.pow(errY, 2));
        eRotate = angle - desAngle;
        lastTime = runtime.milliseconds();

        while(runtime.milliseconds() - driveTime < 500){

            updateCoordinates();

            double theta = Math.atan2(errY, errX);

            if(theta < 0){

                theta += Math.PI;
            }

            theta = -(theta-Math.PI);
            double DeltaTime = (runtime.milliseconds() - lastTime);
            telemetry.addData("DeltaTime", DeltaTime);
            errX = x-desX;
            errY = desY-y;
            refDrive = eDrive;
            refRotate = eRotate;
            eDrive = Math.sqrt(Math.pow(errX, 2) + Math.pow(errY, 2));

            if(errY < 0){

                eDrive *= -1;
            }

            eRotate = angle - desAngle;
            //iDrive; Just reminding myself that these are things
            //dDrive;
            double pDrive; // = 0;
            //iRotate;
            //dRotate;
            double pRotate; // = 0;

            iDrive += eDrive*DeltaTime/10000;

            if(Math.abs(eDrive) > 2){

                iDrive = 0;
            }
            dDrive = (refDrive-eDrive)/DeltaTime;
            pDrive = kDrive[0]*eDrive + kDrive[1]*iDrive + kDrive[2]*dDrive;

            iRotate += eRotate*DeltaTime/10000;

            if(Math.abs(eRotate) > Math.PI/8){

                iRotate = 0;
            }
            dRotate = (refRotate-eRotate)/DeltaTime;
            pRotate = kRotate[0]*eRotate + kRotate[1]*iRotate + kRotate[2]*dRotate;

            telemetry.addData("Proportional", kDrive[0]*eDrive);
            telemetry.addData("Integral", kDrive[1]*iDrive);
            telemetry.addData("Derivative", kDrive[2]*dDrive);
            telemetry.addData("DriveAtTheta", 180*theta/Math.PI);
            telemetry.addData("DriveLinearPower", pDrive);
            telemetry.addData("DriveRotatePower", pRotate);
            telemetry.addData("Error Drive", eDrive);
            telemetry.addData("Error Rotate", eRotate);
            telemetry.update();

            autoDrive(theta, pDrive, pRotate);

            if(Math.abs(eDrive) > driveMargin){
                driveTime = runtime.milliseconds();
            }
            lastTime = runtime.milliseconds();
        }
        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);
    }
    private void autoDrive(double theta, double mag, double Protate){

        double modTheta = theta + Math.PI/4 - angle;

        mag *= (1-Math.abs(Protate)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        double Px = mag * Math.cos(modTheta);
        double Py = mag * Math.sin(modTheta);

        goat.front_left.setPower(Py + Protate);
        goat.back_left.setPower(Px - Protate);
        goat.back_right.setPower(Py - Protate);
        goat.front_right.setPower(Px + Protate);
    }
    private void updateCoordinates(){

        //encoder1.update(goat.encoder1.getVoltage());
        //encoder2.update(goat.encoder2.getVoltage());

        angleOverflow();
        angle = getHeading();

        double DYAngle = encoder2.deltaAngle;
        double DXAngle = -encoder1.deltaAngle;
        double movementAngle = Math.atan2(DYAngle, DXAngle);

        x -= Math.sqrt(Math.pow(DXAngle, 2)+Math.pow(DYAngle, 2))*Math.cos(movementAngle-angle);
        y += Math.sqrt(Math.pow(DXAngle, 2)+Math.pow(DYAngle, 2))*Math.sin(movementAngle-angle);
    }
    private void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0

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
    private double getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion

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

*/