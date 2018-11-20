package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;


public abstract class Red_Capture extends LinearOpMode {
    
    CyGoat goat = new CyGoat();
    
    

    public void driveForward(int speed, double time) {
        
        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);
        
        //sleep(time);
        
    }
    
    public void driveBackward(int speed, double time) {
        
        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);
        
        //sleep(time);
        
    }
    
    public void leftShift(int speed, double time) {
        
        goat.front_left.setPower(-speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(speed);
        
        //sleep(time);
        
    }
    
    public void rightShift(int speed, double time) {
        
        goat.front_left.setPower(speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(-speed);
        
        //sleep(time);
        
    }
    
    public void turnClock(int speed, double time) {
        
        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);
        
        //sleep(time);
        
    }
    
    public void turnCountClock(int speed, double time) {
        
        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);
        
        //sleep(time);
        
    }
    
    public void RosettaStone(int speed, double angle, double rotationSpeed) {
        
        goat.front_left.setPower((speed * Math.sin(-angle + (Math.PI/4)) - rotationSpeed )/ 2);
        goat.front_right.setPower((speed * Math.cos(-angle + (Math.PI/4)) + rotationSpeed )/ 2);
        goat.back_left.setPower((speed * Math.cos(-angle + (Math.PI/4)) - rotationSpeed )/ 2);
        goat.back_right.setPower((speed * Math.sin(-angle + (Math.PI/4)) + rotationSpeed )/ 2);
        
    }
    
    public void detach(int speed, double time) {
        
        
        
    }
    
    public void detachToPoint(int speed, double time, double angle) {
        
        
        
    }
}
