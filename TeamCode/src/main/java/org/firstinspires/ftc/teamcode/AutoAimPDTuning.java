package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AutoAimPDTuning extends OpMode {

    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final FieldCentricDrive fieldCentricDrive = new FieldCentricDrive();

    // ------------------- PD controller ---------------------
    double kP = 0.0020;
    double error = 0;
    double lastError = 0;
    double tgtAngle = 0; // offset here
    double angleTolerance = 0.5;

    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    // ------------------- driving setup ---------------------
    double moveX, moveY, rotX;

    // ------------------- controller based PD tuning -------------------

    double[] stepSizes = {1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        fieldCentricDrive.init(hardwareMap);

        telemetry.addLine("Initialization Complete");
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // ------------------- get mecanum drive inputs ------------------
        moveX = gamepad1.left_stick_x; // strafe
        moveY = gamepad1.left_stick_y; // forward
        rotX = gamepad1.right_stick_x; // rotate

        // ------------------- get april tag info ------------------------
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        // ------------------- auto align rotation logic -----------------

        if (gamepad1.a) {
            if (id20 != null) {
                error = tgtAngle - id20.ftcPose.bearing; // tx (error = where you want to be MINUS where you are)

                if (Math.abs(error) < angleTolerance) {
                    rotX = 0;
                } else {
                    double pTerm = error * kP;
                    curTime = getRuntime();
                    double dT = curTime - lastTime; // change in time
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotX = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastError = 0;
                lastTime = getRuntime();
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        // drive the motors :D
        fieldCentricDrive.drive(moveX, moveY, rotX);
    }
}
