package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AprilTagWebcamTesting", group = "Testing")
public class AprilTagWebcamTesting extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection idRed = aprilTagWebcam.getTagBySpecificId(24);
        AprilTagDetection idBlue = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemtry(idRed);
        aprilTagWebcam.displayDetectionTelemtry(idBlue);

        // telemetry.addData("id20 String", id20.toString()
        /*
        X, Y, Z
        P - pitch, R - roll, Y - yaw
        R - Range (Center of camera to center of tag), B (Angle of deflection away from object), E (Elevation compared to object/AprilTag)
         */

        telemetry.update(); // idk if we need to update, but might as well
    }
}