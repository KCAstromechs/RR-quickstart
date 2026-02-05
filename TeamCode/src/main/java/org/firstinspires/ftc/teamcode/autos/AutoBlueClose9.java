package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AttachmentsRR;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AutoBlueClose9", group = "BlueSide", preselectTeleOp = "Test")
public class AutoBlueClose9 extends LinearOpMode{

    public static class Params {
        public double initialX = 0;
        public double initialY = 0;
        public double initialAngle = Math.toRadians(0);

        public double backwardAmount = 50;
        public double firstBallsAngle = -140;
        public double returnParkAngle = -120;

        public double shootAngle = -30;
//        public double returnAngle = Math.toRadians(returnAngleDeg); // not initial angle bc goofy

        public double tgtRPM = 24;
        public double tgtShootSpeed = .18; // 1.0 = 100%
    }
    public static Params params = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(params.initialX, params.initialY, params.initialAngle);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        AttachmentsRR attachmentsRR = new AttachmentsRR(hardwareMap); // attachmentsRR actions object

//        // TODO (after adding the camera and figuring that out) put vision code here that outputs position
//        int visionOutputPosition = 1;

//        // actions that need to happen on init; for instance, a claw tightening
//        Actions.runBlocking(claw.closeClaw());

//        while (!isStopRequested() && !opModeIsActive()) {
////            int position = visionOutputPosition;
////            telemetry.addData("Position during Init", position);
//        }

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

        // TODO CHANGE Y VALUES TO NEGATIVE AFTER RED IS ADJUSTED
        Action fullPath = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(params.initialX + params.backwardAmount, -(params.initialY))) // back up
                .waitSeconds(5) // shooting
                .turnTo(Math.toRadians(params.firstBallsAngle -3))
                .strafeTo(new Vector2d(params.initialX+params.backwardAmount+10, -(params.initialY+10)))// align with 1st layer of artifacts
                .strafeTo(new Vector2d(params.initialX, -(params.initialY+45))) // also begin intaking
                .strafeTo(new Vector2d(params.initialX+params.backwardAmount-20, -(params.initialY+10)))// scoot back and prepare to get back to shooting pos
                .turnTo(Math.toRadians(params.shootAngle))
                .waitSeconds(5) // wait to shoot
                .turnTo(Math.toRadians(params.firstBallsAngle))
                .strafeTo(new Vector2d(params.initialX+params.backwardAmount+13, -(params.initialY+43))) // line up with next layer of balls
                .strafeTo(new Vector2d(params.initialX+params.backwardAmount-47, -(params.initialY+78))) // also begin intaking
                .strafeTo(new Vector2d(params.initialX+params.backwardAmount-17, -(params.initialY+43))) // scoot to place to shoot
                .turnTo(Math.toRadians(params.shootAngle))
                .waitSeconds(5) // wait to shoot
                .turnTo(Math.toRadians(params.returnParkAngle))
                .strafeTo(new Vector2d(params.initialX-20, -(params.initialY+45)))
                // then move back off line
                .build();

//        Actions.runBlocking(
//                new SequentialAction(
//                    moveBackward,
//                    attachmentsRR.spinUp(params.tgtRPM),
//                    attachmentsRR.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed),
//                    endPath
//                        // add other actions / trajectories
//                )
//        );

        Actions.runBlocking(
                new ParallelAction(
                        attachmentsRR.displayRunTime(telemetry),
                        new ParallelAction(
                                fullPath,
                                new SequentialAction( // the movement
                                        new SleepAction(3), // wait for first movement
                                        attachmentsRR.spinUp(params.tgtRPM),
                                        attachmentsRR.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed),
                                        new SleepAction(2), // wait for alignment to 1st set of balls
                                        attachmentsRR.intake(2), // intake
                                        new SleepAction(4), // wait for repositioning to shoot
                                        attachmentsRR.spinUp(params.tgtRPM),
                                        attachmentsRR.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed),
                                        new SleepAction(3), // wait for alignment to 2nd set of balls
                                        attachmentsRR.intake(2), // intake
                                        new SleepAction(4), // wait for reposition to shoot
                                        attachmentsRR.spinUp(params.tgtRPM),
                                        attachmentsRR.fireArtifact(5, params.tgtRPM, params.tgtShootSpeed)
                                        //fin?
                                )
                        )
                )
        );

    }
}