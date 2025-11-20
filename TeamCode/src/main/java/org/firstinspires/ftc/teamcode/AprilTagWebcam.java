package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerAccess;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer; // DOESN'T exist D:
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    private ExposureControl exposureControl;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;


        // Create/declare/assign aprilTagProcessor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES) // consider changing to inches instead of cm?
                .build();

        // Create/declare/assign visionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();

//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcamName.
//        Camera camera = hardwareMap.get(Camera.class, "Webcam 1");
//        exposureControl = camera.getControl(ExposureControl.class);

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
//        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//

//        telemetry.addData("exposure (i think plz idk man)", exposureControl.getExposure(TimeUnit.MILLISECONDS));
//        telemetry.update();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

//    public ExposureControl getExposureControl() {
//        return exposureControl;
//    }

    public void displayDetectionTelemtry(AprilTagDetection detectedTag) {
        if (detectedTag == null) {return;}
        if (detectedTag.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedTag.id, detectedTag.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedTag.ftcPose.x, detectedTag.ftcPose.y, detectedTag.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedTag.ftcPose.pitch, detectedTag.ftcPose.roll, detectedTag.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedTag.ftcPose.range, detectedTag.ftcPose.bearing, detectedTag.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedTag.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedTag.center.x, detectedTag.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) { // if still an instance of visionPortal, then close
            visionPortal.close();
        }
    }
}