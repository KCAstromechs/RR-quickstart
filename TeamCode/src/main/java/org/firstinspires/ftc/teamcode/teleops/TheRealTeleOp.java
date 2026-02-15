package org.firstinspires.ftc.teamcode.teleops;

// the homemade subsystems
import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LauncherPID;
import org.firstinspires.ftc.teamcode.subsystems.Progression;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

// other other imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "MainTeleOp", group = "Comp")
public class TheRealTeleOp extends OpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private ExposureControl exposureControl;
    private long defaultExposure = 1; // in TimeUnit.MILLISECONDS
    private FieldCentricDrive drive = new FieldCentricDrive();
    private Intake intake = new Intake();
    private LauncherPID launcher = new LauncherPID();
    private Progression progression = new Progression();

    @Override
    public void init() {

        // init main subsystems
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        progression.init(hardwareMap);

        // init camera
        aprilTagWebcam.init(hardwareMap, telemetry);
        while (aprilTagWebcam.getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING){
            // update telemetry while waiting
            telemetry.addData("Camera State", "NOT READY");
            telemetry.update();
        }
        exposureControl = aprilTagWebcam.getVisionPortal().getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(defaultExposure, TimeUnit.MILLISECONDS);
        telemetry.addData("Init Status", "FINISHED");
        telemetry.update();

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        // update camera
        // update the vison portal
        aprilTagWebcam.update();
        AprilTagDetection idRed = aprilTagWebcam.getTagBySpecificId(24);
        AprilTagDetection idBlue = aprilTagWebcam.getTagBySpecificId(20);

        // Attachment Keybinds
        // Intake Keybinds
        if (Math.abs(gamepad1.right_trigger) > .25 || Math.abs(gamepad1.left_trigger) > .25 || gamepad2.x) {
            intake.intake();
        } else if (gamepad1.dpad_down) {
            intake.outtake();
        } else {
            intake.stop();
        }

        // Launcher Keybinds
        if (Math.abs(gamepad2.left_trigger) > 0.25 || Math.abs(gamepad2.right_trigger) > 0.25) {
            launcher.startLauncher();
        } else {
            launcher.stopLauncher();
        }

        // Progression Keybinds
        if (gamepad2.y) {
            progression.manualStopperRaise();
        } else if (gamepad2.x) {
            progression.manualFWD();
        } else if (gamepad2.b) {
            progression.manualBWD();
        } else {
            progression.resetToAuto();
        }

        // Drive Keybinds
        drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x); // movement
        // boost
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            drive.boost();
        } else {
            drive.normal();
        }
        // reset Yaw
        if (gamepad1.y) {
            drive.resetYaw();
        }

        // update subsystems while passing in appropriate args
        drive.updateDrive(); // also pass in aprilTags for future autoaim?

        intake.updateIntake(launcher.getLauncherState());

        progression.updateProgression(launcher.getLauncherState());

        if (idRed != null) {
            launcher.updateLauncher(idRed);
        } else launcher.updateLauncher(idBlue);

        // show main telemetry
        intake.displayTelemetry(telemetry);
        progression.displayTelemetry(telemetry);
        drive.displayTelemetry(telemetry);
        launcher.displayTelemetry(telemetry);

        // show detected tags if any
        for (AprilTagDetection detectedTag : aprilTagWebcam.getDetectedTags()) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedTag.id, detectedTag.metadata.name));
        }
    }
}