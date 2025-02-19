// In this auto, [insert robot dimensions]

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
//@Disabled
@Autonomous(name = "AutoLeft", group = "Autonomous")
public class AutoLeft extends LinearOpMode {


    @Override
    public void runOpMode() { //throws InterruptedException
        // instantiate your MecanumDrive at a particular pose.
        double initialX = 39;
        double initialY = 63.5;
        double initialHeading = Math.toRadians(0);
        String auto_type = "get samples"; // change "park" to "get samples" depending on auto goal, vice versa

        Pose2d initialPose = new Pose2d(initialX, initialY, initialHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        // CREATE RR ATTACHMENT OBJECTS TODO need frontArm
        Lift lift = new Lift(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        FrontArm frontArm = new FrontArm(hardwareMap);

//        // vision here that outputs position
//        int visionOutputPosition = 1;


        // TODO fine tune these numbers / vectors
        Vector2d basketPos = new Vector2d(57, 57);
        double basketAngle = Math.toRadians(45);

        Vector2d sample2Pos = new Vector2d(49, 45);
        double sample2Angle = Math.toRadians(90);

        Vector2d sample3Pos = new Vector2d(60, 45);
        double sample3Angle = Math.toRadians(90);


        Action toP1 = drive.actionBuilder(initialPose) // starts from start
                .strafeTo(basketPos)
                .turnTo(Math.toRadians(basketAngle))
                .build();
                // place first sample into high basket

        Action toP2 = drive.actionBuilder(new Pose2d(basketPos, basketAngle)) // starts from basket
                .turnTo(sample2Angle)
                .strafeTo(sample2Pos)
                .build();
                // Get first ground sample

        Action toP3 = drive.actionBuilder(new Pose2d(sample2Pos, sample2Angle)) // starts from first ground sample
                .strafeTo(basketPos)
                .turnTo(basketAngle)
                .build();
                // place next sample in high basket

        Action toP4 = drive.actionBuilder(new Pose2d(basketPos, basketAngle)) // starts from basket
                .turnTo(sample3Angle)
                .strafeTo(sample3Pos)
                .build();
                // Get second ground sample

        Action toP5 = drive.actionBuilder(new Pose2d(sample3Pos, sample3Angle)) // starts from second ground sample
                .strafeTo(basketPos)
                .turnTo(basketAngle)
                .build();
                // place next sample in high basket

        Action park = drive.actionBuilder(new Pose2d(basketPos, basketAngle)) // starts from basket
                .strafeTo(new Vector2d(51, 7))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(24, 7))
                .build();
                // finish park


        telemetry.addData("Starting Position", initialPose);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        // first, do resets so things don't kaboom
                        new ParallelAction(
                                bucket.reset_bucket(),
                                wrist.wrist_to_sub(),
                                grabber.open_grabber()
                        ),

                        // then, go to high basket and place first sample
                        new ParallelAction(
                                toP1,
                                lift.raise_lift()
                        ),
                        bucket.flip_bucket(),
                        new SleepAction(1),
                        bucket.reset_bucket(), // may need wait time here

                        // then, get first ground sample
                        new ParallelAction(
                                lift.lower_lift(),
                                toP2
                        ),
                        new ParallelAction(
                                wrist.wrist_to_ground(),
                                frontArm.lowerArm(),
                                grabber.open_grabber() // may not need this
                        ),
                        new SleepAction(1),
                        grabber.close_grabber(),
                        new ParallelAction(
                                frontArm.raiseArm(),
                                wrist.wrist_to_bucket()
                        ),
                        new SleepAction(1),
                        grabber.open_grabber(),
                        new SleepAction(0.5),
                        wrist.wrist_to_sub(),

                        // then, put first ground sample into high basket
                        new ParallelAction(
                                lift.raise_lift(),
                                toP3
                        ),
                        bucket.flip_bucket(),
                        new SleepAction(1),
                        bucket.reset_bucket(), // may need wait time here

                        // then, get second ground sample
                        new ParallelAction(
                                lift.lower_lift(),
                                toP4
                        ),
                        new ParallelAction(
                                wrist.wrist_to_ground(),
                                frontArm.lowerArm(),
                                grabber.open_grabber() // may not need this
                        ),
                        new SleepAction(1),
                        grabber.close_grabber(),
                        new ParallelAction(
                                frontArm.raiseArm(),
                                wrist.wrist_to_bucket()
                        ),
                        new SleepAction(1),
                        grabber.open_grabber(),
                        new SleepAction(0.5),
                        wrist.wrist_to_sub(),

                        // then put second ground sample into high basket
                        new ParallelAction(
                                lift.raise_lift(),
                                toP5
                        ),
                        bucket.flip_bucket(),
                        new SleepAction(1),
                        bucket.reset_bucket()//, // may need wait time here

//                        // then park
//                        park

                )
        );

    }
    private void pose_update(Pose2d pose) {
        telemetry.addData("Current Pose: ", pose);
        telemetry.update();
    }

}