// In this auto,

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


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

        // vision here that outputs position
        int visionOutputPosition = 1;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // Whatever the heck we want to happen goes directly below
                .strafeTo(new Vector2d(52, 52))
                .turnTo(Math.toRadians(45));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(52, 52, Math.toRadians(45)))
                // split here and place sample in high basket
//                .strafeTo(new Vector2d(52, 52))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(52, 48));
                // Get first ground sample

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(52, 48, Math.toRadians(90)))
                .strafeTo(new Vector2d(52, 52))
                .turnTo(Math.toRadians(45));
                // place next sample in high basket

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(52, 52, Math.toRadians(45)))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(62, 48));
                // Get second ground sample

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(62, 48, Math.toRadians(90)))
                .strafeTo(new Vector2d(52, 52))
                .turnTo(Math.toRadians(45));
                // place next sample in high basket

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(52, 52, Math.toRadians(45)))
                .strafeTo(new Vector2d(51, 13))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(33, 13));
                // finish park


                /*.strafeTo(new Vector2d(60, 50)) // might not need this
                .strafeTo(new Vector2d(60, 36)) // might not need this
                // THIS IS WHERE WE WILL PICK UP STUFF
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(36, 5))
                .strafeTo(new Vector2d(26, 5))
                .waitSeconds(2);*/

        Action trajectoryActionCloseOut = tab1.fresh()
//                .strafeTo(new Vector2d(48, 12))
//                .turn(Math.toRadians(360))
                .waitSeconds(1)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

//        Action trajectoryActionChosen;
//
//        if (auto_type == "get samples") {
//            trajectoryActionChosen = tab1.build();
//        } else if (auto_type == "park") {
//            trajectoryActionChosen = tab2.build();
//        } else {
//            trajectoryActionChosen = tab2.build();
//        }

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()

//                        grabber.close_grabber(),
//                        grabber.open_grabber(),
//                        lift.raise_lift(),
//                        tab3.build()   //,
//                        trajectoryActionCloseOut
                )
        );

    }
    private void pose_update(Pose2d pose) {
        telemetry.addData("Current Pose: ", pose);
        telemetry.update();
    }

}