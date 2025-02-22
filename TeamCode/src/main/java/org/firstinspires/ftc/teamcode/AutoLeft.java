// In this auto, robot width w/ wheels: 15, w/o wheels: 11.5, length: 17.5 (all inches)
//                                      7.5             5.75          8.75

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

    public static class Params {
        public double basketX = 62;
        public double basketY = 54;

        public double basketAngle = 35;
        public double basketAngle2 = 45;
        public double basketAngle3 = 45;
        public double basketAngle4 = 45;

        public double sample2X = 55;
        public double sample2Y = 52;
        public double sample2Angle = 90;

        public double sample3X = 65;
        public double sample3Y = 52;
        public double sample3Angle = 90;

        public double sample4X = 52.5;
        public double sample4Y = 44;
        public double sample4Angle = 135;
    }

    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() { //throws InterruptedException
        // instantiate your MecanumDrive at a particular pose.
        double initialX = 40;
        double initialY = 64.5;
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
        Vector2d basketPos = new Vector2d(PARAMS.basketX, PARAMS.basketY);
        double basketAngle = Math.toRadians(PARAMS.basketAngle);
        double basketAngle2 = Math.toRadians(PARAMS.basketAngle2);
        double basketAngle3 = Math.toRadians(PARAMS.basketAngle3);
        double basketAngle4 = Math.toRadians(PARAMS.basketAngle4);

        Vector2d sample2Pos = new Vector2d(PARAMS.sample2X, PARAMS.sample2Y);
        double sample2Angle = Math.toRadians(PARAMS.sample2Angle);

        Vector2d sample3Pos = new Vector2d(PARAMS.sample3X, PARAMS.sample3Y);
        double sample3Angle = Math.toRadians(PARAMS.sample3Angle);

        Vector2d sample4Pos = new Vector2d(PARAMS.sample4X, PARAMS.sample4Y);
        double sample4Angle = Math.toRadians(PARAMS.sample4Angle);



        Action toP1 = drive.actionBuilder(initialPose) // starts from start
                .strafeTo(basketPos)
                .turnTo(basketAngle)
                .build();
                // place first sample into high basket

        Action toP2 = drive.actionBuilder(new Pose2d(basketPos, basketAngle)) // starts from basket
                .turnTo(sample2Angle)
                .strafeTo(sample2Pos)
                .build();
                // Get first ground sample

        Action toP3 = drive.actionBuilder(new Pose2d(sample2Pos, sample2Angle)) // starts from first ground sample
                .strafeTo(basketPos)
                .turnTo(basketAngle2)
                .build();
                // place next sample in high basket

        Action toP4 = drive.actionBuilder(new Pose2d(basketPos, basketAngle)) // starts from basket
                .turnTo(sample3Angle)
                .strafeTo(sample3Pos)
                .build();
                // Get second ground sample

        Action toP5 = drive.actionBuilder(new Pose2d(sample3Pos, sample3Angle)) // starts from second ground sample
                .strafeTo(basketPos)
                .turnTo(basketAngle3)
                .build();
                // place next sample in high basket

        Action toP6 = drive.actionBuilder(new Pose2d(basketPos, basketAngle))
                .strafeTo(sample4Pos)
                .turnTo(sample4Angle)
                .build();

        Action toP7 = drive.actionBuilder(new Pose2d(sample4Pos, sample4Angle))
                .strafeTo(basketPos)
                .turnTo(basketAngle4)
                .build();

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
//                      First, go to high basket and place first sample (and do initialization)
                        new ParallelAction(
                                bucket.reset_bucket(),
                                wrist.wrist_to_sub(),
                                grabber.open_grabber(),
                                toP1,
                                lift.raise_lift()
                        ),
                        bucket.flip_bucket(),
                        new SleepAction(1),
                        bucket.reset_bucket(), // may need wait time here

                        // then, get first ground sample
                        new ParallelAction(
                                lift.lower_lift(),
                                toP2,
                                wrist.wrist_to_ground(),
                                grabber.open_grabber()
                        ),
                        frontArm.lowerArm(),
                        new SleepAction(.7),
                        grabber.close_grabber(),
                        new SleepAction(.5),
                        new ParallelAction(
                                frontArm.raiseArm(),
                                wrist.wrist_to_bucket()
                        ),
                        new SleepAction(.2),
                        grabber.open_grabber(),
                        new SleepAction(.5),
                        wrist.wrist_to_sub(),
//                        new SleepAction(.5),

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
                                wrist.wrist_to_ground(),
                                grabber.open_grabber(), // may not need this
                                lift.lower_lift(),
                                toP4
                        ),
                        frontArm.lowerArm(),
                        new SleepAction(.7),
                        grabber.close_grabber(),
                        new SleepAction(.5),
                        new ParallelAction(
                                frontArm.raiseArm(),
                                wrist.wrist_to_bucket()
                        ),
                        new SleepAction(.2),
                        grabber.open_grabber(),
                        new SleepAction(.5),
                        wrist.wrist_to_sub(),

                        // then put second ground sample into high basket
                        new ParallelAction(
                                lift.raise_lift(),
                                toP5
                        ),
                        bucket.flip_bucket(),
                        new SleepAction(1),
                        bucket.reset_bucket(), // may need wait time here

                        // then grab the third ground sample
                        new ParallelAction(
                                wrist.wrist_to_ground(),
                                grabber.open_grabber(), // may not need this
                                lift.lower_lift(),
                                toP6
                        ),
                        frontArm.lowerArm(),
                        new SleepAction(.7),
                        grabber.close_grabber(),
                        new SleepAction(.5),
                        new ParallelAction(
                                frontArm.raiseArm(),
                                wrist.wrist_to_bucket()
                        ),
                        new SleepAction(.2),
                        grabber.open_grabber(),
                        new SleepAction(.5),
                        wrist.wrist_to_sub(),

                        // then put the third ground sample into the high basket
                        new ParallelAction(
                                lift.raise_lift(),
                                toP7
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