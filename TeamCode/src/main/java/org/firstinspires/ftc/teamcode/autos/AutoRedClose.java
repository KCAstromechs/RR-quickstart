package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Attachments;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AutoRedClose", group = "RedSide", preselectTeleOp = "Test")
public class AutoRedClose extends LinearOpMode{

    public static class Params {
        public double initialX = 0;
        public double initialY = 0;
        public double initialAngle = Math.toRadians(0);

        public double backwardAmount = 40;
        public double turnAngle = Math.toRadians(-219);
        public double tgtRPM = 24;
        public double tgtShootSpeed = .23; // 1.0 = 100%
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

        while (!isStopRequested() && !opModeIsActive()) {
//            int position = visionOutputPosition;
//            telemetry.addData("Position during Init", position);
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
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(params.backwardAmount);

        Action endPath = drive.actionBuilder(new Pose2d(params.backwardAmount, 0, Math.toRadians(0)))
                .turnTo(params.turnAngle)
                .lineToX(params.initialX+5)
                .build();

        Action moveBackward;
        moveBackward = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                    moveBackward,
                    attachments.spinUp(params.tgtRPM),
                    attachments.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed),
                    endPath
                        // add other actions / trajectories
                )
        );
    }
}
