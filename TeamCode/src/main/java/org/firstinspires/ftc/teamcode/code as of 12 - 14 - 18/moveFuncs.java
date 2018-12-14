package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class moveFuncs {

    public moveFuncs() {}

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

    public void detach() {

        moveArm(-0.7, 500);
        rightShift(0.3, 250);
        driveForward(0.5, 100);
        leftShift(0.3, 250);

    }

    void moveArm(double speed, long time) {

        goat.arm_motor_1.setPower(speed);
        goat.arm_motor_2.setPower(speed);

        sleep(time);

    }

    public void moveArm(double speed) {

        goat.arm_motor_1.setPower(speed);
        goat.arm_motor_2.setPower(speed);

    }

    public void clampPosition() {

        double armVolt = (float) goat.potentiometer.getVoltage();

        while (armVolt < 3.28) {

            moveArm(1);

        }
    }

    public void openPosition() {

        double armVolt = (float) goat.potentiometer.getVoltage();

        while (armVolt > 3.28) {

            moveArm(-1);

        }

    }

    public void vertPosition() {

        double armVolt = (float) goat.potentiometer.getVoltage();

        while (armVolt <  1.5) {

            double currentVolt = (float) goat.potentiometer.getVoltage();
            moveArm(-1);
            if (currentVolt > 1.5) {

                break;

            }

        }

        while (armVolt >  1.5) {

            double currentVolt = (float) goat.potentiometer.getVoltage();
            moveArm(-1);
            if (currentVolt < 1.5) {

                break;

            }

        }

        while (armVolt ==  1.5) {

            break;

        }


    }

    public void landerPosition() {

        double armVolt = (float) goat.potentiometer.getVoltage();

        while (armVolt <  1.7) {

            double currentVolt = (float) goat.potentiometer.getVoltage();
            moveArm(-1);
            if (currentVolt > 1.7) {

                break;

            }

        }

        while (armVolt >  1.7) {

            double currentVolt = (float) goat.potentiometer.getVoltage();
            moveArm(-1);
            if (currentVolt < 1.7) {

                break;

            }

        }

        while (armVolt ==  1.7) {

            break;

        }

        moveArm(1);

    }

    void releaseMarker(double speed) {

        goat.marker.setPosition(1);
        sleep(250);
        goat.marker.setPosition(0);

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
