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


        // CREATE RR ATTACHMENT OBJECTS
        Lift lift = new Lift(hardwareMap);
        Grabber grabber = new Grabber(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // Whatever the heck we want to happen goes directly below
                .strafeTo(new Vector2d(53, 61))
                .turnTo(Math.toRadians(45))
                // split here and place sample in high basket
                .strafeTo(new Vector2d(52, 52))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(53, 61))
                .turnTo(Math.toRadians(45));

                /*.strafeTo(new Vector2d(60, 50)) // might not need this
                .strafeTo(new Vector2d(60, 36)) // might not need this
                // THIS IS WHERE WE WILL PICK UP STUFF
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(36, 5))
                .strafeTo(new Vector2d(26, 5))
                .waitSeconds(2);*/
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(54, 61.5, initialHeading))
                .turnTo(Math.toRadians(45));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20, 11));

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

        Action trajectoryActionChosen;

        if (auto_type == "get samples") {
            trajectoryActionChosen = tab1.build();
        } else if (auto_type == "park") {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab2.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen

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