package org.firstinspires.ftc.teamcode.autos;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Attachments;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AutoFar", group = "Any Side", preselectTeleOp = "Test")
public class AutoFar extends LinearOpMode{

    public static class Params {
        public double initialX = 0;
        public double initialY = 0;
        public double initialAngle = Math.toRadians(0);

        public double turnAimAngle = 10;
        public double backwardAmount = 20;

        public double defaultRPM = 50;
        public double defaultSpeed = 50;
    }
    public static Params params = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(params.initialX, params.initialY, params.initialAngle);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Attachments attachments = new Attachments(hardwareMap); // attachments actions object

//        // TODO (after adding the camera and figuring that out) put vision code here that outputs position
//        int visionOutputPosition = 1;

//        // actions that need to happen on init; for instance, a claw tightening
//        Actions.runBlocking(claw.closeClaw());
        double targetRPM = params.defaultRPM;
        double targetSpeed = params.defaultSpeed;

        while (!isStopRequested() && !opModeIsActive()) {
//            int position = visionOutputPosition;
//            telemetry.addData("Position during Init", position);
            if (gamepad1.dpadUpWasPressed()) {
                targetRPM += 5;
                targetSpeed += 5;
            } else if (gamepad1.dpadDownWasPressed()) {
                targetRPM -= 5;
                targetSpeed -= 5;
            }

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target Speed", targetSpeed);
            telemetry.addData("Initialization Status", "Initializing?");
            telemetry.update();
        }

//        int startPosition = visionOutputPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // change trajectory if needed here with if statements
        Action moveForward = drive.actionBuilder(initialPose)
                .lineToX(-params.backwardAmount)
                .build();

        Action leaveWall = drive.actionBuilder(initialPose)
                .lineToX(-10)
                .turnTo(Math.toRadians(params.turnAimAngle))
                .build();

        Actions.runBlocking(
                new SequentialAction(
//                        leaveWall,
                        attachments.spinUp(50),
                        attachments.fireArtifact(5, targetRPM, targetSpeed
























                        ),//, speed change to 45?
                        moveForward
                        // add other actions / trajectories
                )
        );
    }
}
