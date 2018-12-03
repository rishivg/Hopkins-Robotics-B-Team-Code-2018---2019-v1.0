package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import java.util.Map;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Hermione Tele Op", group="Tele Op")

public class Hermione extends LinearOpMode {

    CyGoat goat = new CyGoat();

    ToggleMap togMap1 = new ToggleMap();
    MauraudersMap maurMap1 = new MauraudersMap();

    ToggleMap togMap2 = new ToggleMap();
    MauraudersMap maurMap2 = new MauraudersMap();

    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S *////* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////

    /*IMU Variables*/
    double newZero = 0;
    int fullRotationCount = 0;
    double previousAngle = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        goat.init(hardwareMap);
        goat.imu();

        while(!opModeIsActive()){
        }
        while(opModeIsActive()){
            angleOverflow(); //Keep at the beginning of teleop loop
            drive();
            //intake();
            arm();
            armClamps();
            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }

    // DRIVE CODE //

    public void drive(){

        double Protate = 0.6 * gamepad1.right_stick_x;
        double newProtate = 0.6 * -gamepad1.right_stick_x;
        double stick_x = -gamepad1.left_stick_y; //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = gamepad1.left_stick_x; //Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2)
        double gyroAngle = getHeading(); //In radiants, proper rotation, yay!!11!!
        double magnitudeMultiplier = 0;

        if(gamepad1.left_bumper){ //Removes gyroAngle from the equation meaning the robot drives normally
            gyroAngle = 0;
        }

        double theta = Math.atan2(stick_y, stick_x); //Arctan2 doesn't have bad range restriction
        double modifiedTheta = theta + Math.PI/4 - gyroAngle;

        double thetaInFirstQuad = Math.abs(Math.atan(stick_y/stick_x)); //square to circle conversion
        if(thetaInFirstQuad > Math.PI/4){
            magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
        }
        else if(thetaInFirstQuad <= Math.PI/4){
            magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
        }

        double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2))*magnitudeMultiplier*(1-Math.abs(Protate)); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        double Px = magnitude * Math.cos(modifiedTheta);
        double Py = magnitude * Math.sin(modifiedTheta);

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Theta", theta);
        telemetry.addData("Modified Theta", modifiedTheta);
        telemetry.addData("Magnitude",  magnitude);
        telemetry.addData("Front Left", Py + Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py - Protate);
        telemetry.addData("Front Right", Px + Protate);

        if(gamepad1.dpad_right || (togMap1.right_bumper && theta > Math.PI/4 && theta <= 3*Math.PI/4)){
            Px = -1;
            Py = 1;
        }
        else if(gamepad1.dpad_down || (togMap1.right_bumper && (theta > 3*Math.PI/4 || theta <= -3*Math.PI/4))){
            Px = -1;
            Py = -1;
        }
        else if(gamepad1.dpad_left || (togMap1.right_bumper && theta < -Math.PI/4 && theta >= -3*Math.PI/4)){
            Px = 1;
            Py = -1;
        }
        else if(gamepad1.dpad_up || (togMap1.right_bumper && theta > -Math.PI/4 && theta <= Math.PI/4 && !(stick_y == 0 && stick_x == 0))){
            Px = 1;
            Py = 1;
        }
        else if(togMap1.right_bumper){
            Px = 0;
            Py = 0;
        }
        goat.front_left.setPower(Py + Protate);
        goat.back_left.setPower(Px + newProtate);
        goat.back_right.setPower(Py -   Protate);
        goat.front_right.setPower(Px - newProtate);
    }

    public void intake() {

        goat.Intake1.setPower(gamepad2.right_trigger + (gamepad2.left_trigger * -1));
        //goat.Intake2.setPower(gamepad2.right_trigger + (gamepad2.left_trigger * -1));

        telemetry.addData("Intake Power:", (gamepad2.right_trigger + (gamepad2.left_trigger * -1)));
    }

    public void arm() {

        goat.arm_motor.setPower(gamepad2.left_stick_y);

        telemetry.addData("Arm Power:", gamepad2.left_stick_y);

    }

    public void armClamps() {

        if (gamepad2.dpad_left) {

            goat.left_clamp.setPosition(1);
            goat.right_clamp.setPosition(1);

        } else if (gamepad2.dpad_right) {

            goat.left_clamp.setPosition(0);
            goat.right_clamp.setPosition(0);

        }

    }

    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0
        double heading = getHeading() - fullRotationCount*(2*Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if(heading < Math.PI/4 && previousAngle > 3*Math.PI/4){
            fullRotationCount++;
        }
        if(heading > 3*Math.PI/4 && previousAngle < Math.PI/4){
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
        heading = heading - newZero;

        heading += fullRotationCount*(2*Math.PI);
        return heading;
    }

    // TOGGLES ////////// USE MAP //

    public void updateKeys(){
        if(gamepad1.a && cdCheck(maurMap1.a, 1000)){
            togMap1.a = toggle(togMap1.a);
            maurMap1.a = runtime.milliseconds();
        }
        if(gamepad1.b && cdCheck(maurMap1.b, 500)){
            togMap1.b = toggle(togMap1.b);
            maurMap1.b = runtime.milliseconds();
        }
        if(gamepad2.b && cdCheck(maurMap2.b, 500)){
            togMap2.b = toggle(togMap2.b);
            maurMap2.b = runtime.milliseconds();
        }
        if(gamepad1.right_stick_x > 0 && cdCheck(maurMap1.right_stick_right, 700)){
            togMap1.right_stick_right = toggle(togMap1.right_stick_right);
            maurMap1.right_stick_right = runtime.milliseconds();
        }
    }

    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }
}
