package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class moveFuncs {


    CyGoat goat = new CyGoat();

    private void driveForward(double speed, long time) {

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        sleep(time);

    }

    public void driveBackward(double speed, long time) {

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

        sleep(time);

    }

    private void leftShift(double speed, long time) {

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(speed);

        sleep(time);

    }

    private void rightShift(double speed, long time) {

        goat.front_left.setPower(speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(-speed);

        sleep(time);

    }

    public void turnClock(double refSpeed) {

        double speed = refSpeed;

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

    }

    public void turnClock(double speed, long time) {

        goat.front_left.setPower(speed);
        goat.back_left.setPower(speed);
        goat.back_right.setPower(-speed);
        goat.front_right.setPower(-speed);

        sleep(time);

    }

    public void turnCountClock(double speed) {

        goat.front_left.setPower(-1 * speed);
        goat.back_left.setPower(-1 * speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

    }

    public void stopMotors(long time) {

        goat.front_left.setPower(0);
        goat.back_left.setPower(0);
        goat.back_right.setPower(0);
        goat.front_right.setPower(0);

        sleep(time);

    }

    private void turnCountClock(double speed, long time) {

        goat.front_left.setPower(-speed);
        goat.back_left.setPower(-speed);
        goat.back_right.setPower(speed);
        goat.front_right.setPower(speed);

        sleep(time);

    }

    public void RosettaStone(double speed, double angle, double rotationSpeed) {

        goat.front_left.setPower((speed * Math.sin(-angle + (Math.PI/4)) - rotationSpeed )/ 2);
        goat.front_right.setPower((speed * Math.cos(-angle + (Math.PI/4)) + rotationSpeed )/ 2);
        goat.back_left.setPower((speed * Math.cos(-angle + (Math.PI/4)) - rotationSpeed )/ 2);
        goat.back_right.setPower((speed * Math.sin(-angle + (Math.PI/4)) + rotationSpeed )/ 2);

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
