package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "Concept: VuMark Id", group = "Concept")
@Disabled
public class ConceptLinearOp extends LinearOpMode {

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AfVQDvr/////AAAAmZELbx83XkYGp459tCxRasRQunZxW6g2bjgEZA4injyHVebk3wY00yDXFnUkJNFGZQQ3w6qSrlv8hxOIqL9CcP93xUiB/bz+XvlxfaR6v/2Hb5mTCWmue3XsKIxywuy7/XRm7Ttq1MduxAY5JtADMDCmDa4yXiTtLYU6w4NEXW/SjoUoOhC3+vlBVUtMa6Qe7cwms2UjlnljtvsbySIe2zf22n4R797WX6oWLtU4QfkmnX3CVUKAh0kXawTfjlR/IZFKSdmjqo7Ux5fZK2GUDdaQlfngDfSf1jm8Q+tfk6G5rO072vERWRUxQv76B1uTXRwMuLsFZKTy9tAaWUsFzfifWe01/VGiYnWbtEUFnmnW";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                if (vuMark == RelicRecoveryVuMark.RIGHT)
                    telemetry.addData("VuMark", "Right");
                else if (vuMark == RelicRecoveryVuMark.CENTER)
                    telemetry.addData("VuMark", "Center");
                else if (vuMark == RelicRecoveryVuMark.LEFT)
                    telemetry.addData("VuMark", "Left");
                else if (vuMark == RelicRecoveryVuMark.UNKNOWN)
                    telemetry.addData("VuMark", "Unknown");

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

}

