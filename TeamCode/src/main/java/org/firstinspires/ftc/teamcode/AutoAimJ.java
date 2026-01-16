package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AutoAimJ {
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private ArrayList<AprilTagDetection> detectedTags = new ArrayList<>();
    private OpMode op;
    private Telemetry telemetry;

    public static boolean autoAimRequested;
    public static Boolean autoDistancingRequested;
    public Boolean targetLocked;

    double bearing;
    double yaw;
    double range;

    double minimumBoundry = -3;
    double maximumBoundry = 3;

    double SPEED_GAIN = .3;
    double turnPower;

    private Servo indicatorLight;

    public void init(OpMode OP) {
        op = OP;
        telemetry = op.telemetry;

        indicatorLight = op.hardwareMap.get(Servo.class, "LED");

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        processor.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(op.hardwareMap.get(WebcamName.class, "webCam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);
        builder.addProcessors(processor);
        portal = builder.build();
    }

    public void loop(){
        update();
        getTagInfo();
        AprilTagDetection id20 = getTagBySpecificId(20);
        displayDetectionTelemetry(id20);

        autoAim();
        manageIndicatorLight();
    }
    public void stop(){
        if (portal != null) {
            portal.close();
        }}

    public void autoAim(){
        if (autoAimRequested) {
            turnPower = bearing * SPEED_GAIN;
            Shooter.turretRotator.setPower(turnPower);

            if (bearing > minimumBoundry && bearing < maximumBoundry){
                Shooter.turretRotator.setPower(0);
                targetLocked = true;
            } else {
                targetLocked = false;
            }
        }
    }
    public void manageIndicatorLight(){
        if (targetLocked){
            indicatorLight.setPosition(.5);
        } else {
            indicatorLight.setPosition(.2777);
        }
    }

    /*  X = The green X axis value represents the sideways offset to the tag.
        Note that this value is negative (to the left of the camera center).

        Y = The red Y axis value represents the forward distance to the Tag.

        YAW: The cyan Yaw value represents the rotation of the tag around the Z axis.
        A Counter-Clockwise rotation is considered positive.
        Note that a Yaw value of zero means that the tag image is parallel to the face of the camera.

        RANGE: Range (which is the direct distance to the center of the target)
        BEARING: Bearing (which is how many degrees the camera must turn to point directly at the target)
        Elevation: (which is how many degrees the camera must tilt UP to center on the tag)
    */








    public void update(){
        detectedTags = processor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return  detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId){
        if (detectedId == null){return;}

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }
    public AprilTagDetection getTagBySpecificId(int id){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getTagInfo(){
        for (AprilTagDetection detection : detectedTags){
            bearing = detection.ftcPose.elevation;
            yaw = detection.ftcPose.yaw;
            range = detection.ftcPose.range;
        }
        return null;
    }

    }
