package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

@Config
//@Disabled
@Autonomous(name = "AutoRight", preselectTeleOp = "TELELEOPTETESTING (Java)")
public class AutoRight extends LinearOpMode {

//    public class Lift {
//        private DcMotorEx lift;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//    }

    @Override
    public void runOpMode() { //throws InterruptedException
        // instantiate your MecanumDrive at a particular pose.
        double initialX = -13;
        double initialY = 63.5;
        double initialHeading = Math.toRadians(180);
        String auto_type = "get samples"; // change "park" to "get samples" depending on auto goal, vice versa

        Pose2d initialPose = new Pose2d(initialX, initialY, initialHeading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // make attachments

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                // Whatever the heck we want to happen goes directly below
                .strafeTo(new Vector2d(-37, 37))
                .strafeTo(new Vector2d(-37, 5))
                .strafeTo(new Vector2d(-47, 5))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-47, 47))
                .strafeTo(new Vector2d(-47, 5))
                .strafeTo(new Vector2d(-59, 5))
                .strafeTo(new Vector2d(-59, 47))
                .strafeTo(new Vector2d(-59, 20))
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(-59, 47))
                .waitSeconds(2);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-33, 63.5))
                .waitSeconds(2);
        Action trajectoryActionCloseOut = tab1.fresh()
//                .strafeTo(new Vector2d(48, 12))
//                .turn(Math.toRadians(360))
                .strafeTo(new Vector2d(initialX, initialY))
                .waitSeconds(1)
                .build();
        

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
                        trajectoryActionChosen//,
//                        trajectoryActionCloseOut
                )
        );

    }
    private void pose_update(Pose2d pose) {
        telemetry.addData("Current Pose: ", pose);
        telemetry.update();
    }

}