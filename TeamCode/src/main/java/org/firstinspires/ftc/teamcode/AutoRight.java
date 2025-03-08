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
@Autonomous(name = "AutoRight", group = "competition", preselectTeleOp = "TELELEOPTETESTING (Java)")
public class AutoRight extends LinearOpMode {

    public static class Params {

        public double firstSpotX = -37;
        public double firstSpotY = 37;
        public double secondSpotX = -37;
        public double secondSpotY = 5;

        public double obsZoneY = 50;

        public double sample1X = -47;
        public double sample1Y = 5;

        public double sample2X = -57;
        public double sample2Y = 5;

        public double sample3X = -65;
        public double sample3Y = 5;

    }

    public static Params PARAMS = new Params();

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

        Action tab1 = drive.actionBuilder(initialPose)
                // Whatever the heck we want to happen goes directly below
                .strafeTo(new Vector2d(PARAMS.firstSpotX, PARAMS.firstSpotY)) // to first spot
                .strafeTo(new Vector2d(PARAMS.secondSpotX, PARAMS.secondSpotY)) // to second spot
                .strafeTo(new Vector2d(PARAMS.sample1X, PARAMS.sample1Y)) // to first sample
                .turnTo(Math.toRadians(90)) // AnglE
                .strafeTo(new Vector2d(PARAMS.sample1X, PARAMS.obsZoneY)) // bring first sample back
                .strafeTo(new Vector2d(PARAMS.sample1X, PARAMS.sample1Y)) // back to first sample area
                .strafeTo(new Vector2d(PARAMS.sample2X, PARAMS.sample2Y)) // to second sample
                .strafeTo(new Vector2d(PARAMS.sample2X, PARAMS.obsZoneY)) // bring back second sample
                .strafeTo(new Vector2d(PARAMS.sample2X, PARAMS.sample2Y)) // back to second sample area for third sample
                .strafeTo(new Vector2d(PARAMS.sample3X, PARAMS.sample3Y)) // to third sample
                .strafeTo(new Vector2d(PARAMS.sample3X, PARAMS.obsZoneY)) // bring back third sample
                .waitSeconds(2)
                .build();
        

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1;

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );

    }
}