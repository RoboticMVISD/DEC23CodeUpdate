
package org.firstinspires.ftc.teamcode;

import android.util.Size;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;
public class AutoAim {

    private static final boolean USE_WEBCAM = true;

    static OpMode op;

    private static AprilTagProcessor sigmaAprilTag;

    private static VisionPortal sigmaVisionPortal;

    private static String currentObeliskTag;
    private static int currentGoalTag;
    private static double rotateDuration;
    private static double currentGoalElevation;
    private static double currentGoalRange;
    private static double targetLaunchVelocity;
    private static boolean noTagDetected;
    public static boolean targetLocked = false;
    public static boolean aimEnabled = false;
    public static boolean launcherRequested = false;
    public static boolean refindGoalAttempted = true;

    static final private double turnMultiplier = 2.5;
    static final private double launchMultiplier = 6.8;
    static final private double launchOffset = 756.68282;
    static final private double closeBoundary = 2;
    static final private double turnPower = 1;


    /**
     * The variables to store our instances of the turret rotator, shooters, and the indicator light.
     */
    private static Servo indicatorLight;

    public static void init(OpMode OP) {
        op = OP;

        indicatorLight = op.hardwareMap.get(Servo.class, "LED");
        initAprilTag();

        op.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        op.telemetry.addData(">", "Touch START to start OpMode");
    }

    public static void loop() {

        telemetryAprilTag();
        aimTurret();
        autoDistance();

        op.telemetry.addData("Current Goal", currentGoalTag);
        op.telemetry.addData("Current Obelisk", currentObeliskTag);
        op.telemetry.addData("Current Bearing", currentGoalElevation);

        targetLaunchVelocity = ((launchMultiplier * currentGoalRange) + launchOffset);

        op.telemetry.addData("Auto Launch Speed", targetLaunchVelocity);

        op.telemetry.addData("Target locked", targetLocked);
    }

    public static void aimTurret(){
        if (aimEnabled) {
            try {
                positionTurret();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else {
            Shooter.turretRotator.setPower(0);
            indicatorLight.setPosition(0.388);
        }
    }

    public static void autoDistance(){
        if (launcherRequested) {
            setLauncherPower();
        } else {
            Shooter.leftShooter.setVelocity(0);
            Shooter.rightShooter.setVelocity(0);
        }
    }

    private static void initAprilTag() {

        // Create the AprilTag processor.
        AprilTagProcessor.Builder sigmaAprilTagBuilder = new AprilTagProcessor.Builder();

        // The following default settings are available to un-comment and edit as needed.
        sigmaAprilTagBuilder.setDrawAxes(true);
        sigmaAprilTagBuilder.setDrawCubeProjection(false);
        sigmaAprilTagBuilder.setDrawTagOutline(true);
        sigmaAprilTagBuilder.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11);
        sigmaAprilTagBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        sigmaAprilTagBuilder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);

        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.

        sigmaAprilTag = sigmaAprilTagBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        sigmaAprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder sigmaVisionPortalBuilder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            sigmaVisionPortalBuilder.setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            sigmaVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        sigmaVisionPortalBuilder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        sigmaVisionPortalBuilder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        sigmaVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        sigmaVisionPortalBuilder.setAutoStopLiveView(false);

        // Set and enable the processor.
        sigmaVisionPortalBuilder.addProcessor(sigmaAprilTag);

        // Build the Vision Portal, using the above settings.
        sigmaVisionPortal = sigmaVisionPortalBuilder.build();

        // Disable or re-enable the aprilTag processor at any time.
        sigmaVisionPortal.setProcessorEnabled(sigmaAprilTag, true);

    }
    private static void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = sigmaAprilTag.getDetections();
        //op.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                /*op.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                op.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                op.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                op.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));*/
                if (detection.id == 21){
                    currentObeliskTag = "GPP";
                } else if (detection.id == 22){
                    currentObeliskTag = "PGP";
                } else if (detection.id == 23){
                    currentObeliskTag = "PPG";
                } else {
                    currentGoalTag = detection.id;
                    currentGoalElevation = detection.ftcPose.elevation;
                    currentGoalRange = detection.ftcPose.range;

                    op.telemetry.addData("Goal Distance", currentGoalRange);
                }
            } else {
                //op.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                //op.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        noTagDetected = currentDetections.isEmpty();

        op.telemetry.addData("Tag Detected", !noTagDetected);

        // Add "key" information to telemetry
        //op.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //op.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //op.telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    /**
     * using the turret rotator, position the turret to the proper horizontal angle for the goal.
     */
    public static void positionTurret() throws InterruptedException {

        rotateDuration = (Math.abs(currentGoalElevation) / turnPower) * turnMultiplier;

        if((currentGoalElevation != -1) && !noTagDetected){
            if (currentGoalElevation > 0) {
                Shooter.turretRotator.setPower(-turnPower);
            } else {
                Shooter.turretRotator.setPower(turnPower);
            }
            Thread.sleep((long) rotateDuration);
            Shooter.turretRotator.setPower(0);

            refindGoalAttempted = false;
        } else if (noTagDetected && !refindGoalAttempted){

            Shooter.turretRotator.setPower(0);

            refindGoalAttempted = true;
        } else {
            Shooter.turretRotator.setPower(0);
        }

        targetLocked = currentGoalElevation < closeBoundary && currentGoalElevation > -closeBoundary && !noTagDetected;

        if (targetLocked) {
            indicatorLight.setPosition(0.500);
        } else {
            indicatorLight.setPosition(0.290);
        }
    }

    private static void setLauncherPower() {
        Shooter.leftShooter.setVelocity(targetLaunchVelocity);
        Shooter.rightShooter.setVelocity(targetLaunchVelocity);
    }

}

 /*if (currentGoalElevation > 0) {
                bearingServo.setPower(turnPower);
            } else {
                bearingServo.setPower(-turnPower);
            }
;
            Thread.sleep(250);*/