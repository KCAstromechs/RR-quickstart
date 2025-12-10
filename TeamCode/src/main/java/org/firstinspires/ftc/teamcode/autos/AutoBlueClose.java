package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

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
@Autonomous(name = "AutoBlueClose", group = "RedSide", preselectTeleOp = "Test")
public class AutoBlueClose extends LinearOpMode{

    public static class Params {
        public double initialX = 0;
        public double initialY = 0;
        public double initialAngle = Math.toRadians(0);

        public double backwardAmount = 50;
        public double turnAngle = Math.toRadians(219);
        public double tgtRPM = 30;
        public double tgtShootSpeed = .2; // 1.0 = 100%
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
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToX(params.backwardAmount);
//
//        Action endPath = drive.actionBuilder(new Pose2d(params.backwardAmount, 0, Math.toRadians(0)))
//                .turnTo(params.turnAngle)
//                .lineToX(params.initialX+5)
//                .build();

        Action fullPath = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(params.initialX + params.backwardAmount, params.initialY))
                .waitSeconds(5) // shooting
                .turnTo(params.turnAngle)
                .strafeTo(new Vector2d(-(params.initialX+params.backwardAmount+10), params.initialY+10))// align with 1st layer of artifacts
                .strafeTo(new Vector2d(-params.initialX, params.initialY+45)) // also begin intaking
                .strafeTo(new Vector2d(-(params.initialX+params.backwardAmount-20), params.initialY+10))// scoot back and prepare to get back to shooting pos
                .turnTo(Math.toRadians(params.returnAngleDeg))
                .waitSeconds(5) // wait to shoot
                .turnTo(Math.toRadians(100))
                .strafeTo(new Vector2d(-params.initialX, params.initialY+45))
                // then move back off line
                .build();

//        Actions.runBlocking(
//                new SequentialAction(
//                    moveBackward,
//                    attachments.spinUp(params.tgtRPM),
//                    attachments.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed),
//                    endPath
//                        // add other actions / trajectories
//                )
//        );

        Actions.runBlocking(
                new ParallelAction(
                        attachments.displayRunTime(telemetry),
                        new ParallelAction(
                                fullPath,
                                new SequentialAction( // the movement
                                        new SleepAction(3), // wait for first movement
                                        attachments.spinUp(params.tgtRPM),
                                        attachments.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed),
                                        new SleepAction(2), // wait for alignment to 1st set of balls
                                        attachments.intake(1.67), // intake
                                        new SleepAction(4), // wait for repositioning to shoot
                                        attachments.spinUp(params.tgtRPM),
                                        attachments.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed)
                                        // fin ... probably
                                )
                        )
                )
        );
    }

    @NonNull
    public static Action turnLeft(double degrees, MecanumDrive drive) {
        return drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .turnTo(Math.toRadians(degrees))
                .build();
    }
}
