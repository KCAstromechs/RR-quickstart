// THIS IS MARK 3 CODE (Field Centric)

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "TELELEOPTETESTING (Java)", group = "competition")
public class TELELEOPTETESTING extends LinearOpMode {

    // WRIST stuff
    public static class WristParams {
        public double groundPosition = 0.75;
        public double subPosition = 0.9;
        public double bucketPosition = 0.55;
    }
    // LIFT stuff
    public static class LiftParams {
        public double upPosition = 4000;
        public double downPosition = 50;
        public double speedLimitDown = 0.5; // 50% power/speed
        public double speedLimitUp = 1; // 100% power/speed
    }
    // GRABBER stuff
    public static class GrabberParams {
        public double closedPosition = 0.9;
        public double openPosition = 0.65;
    }
    // BUCKET stuff
    public static class BucketParams {
        public double flippedPosition = 0.1;
        public double normalPosition = 0.67;
    }
    // FRONT ARM stuff
    public static class FrontArmParams {
        public double downPosition = 240;
        public double upPosition = 20;
        public double speedLimit = 0.5;  // 50% power/speed
        public double autoRangeLimPos = 50;
    }

    // Paramamamamameters
    public static WristParams WRISTPARAMS = new WristParams();
    public static LiftParams LIFTPARAMS =  new LiftParams();
    public static GrabberParams GRABBERPARAMS = new GrabberParams();
    public static BucketParams BUCKETPARAMS = new BucketParams();
    public static FrontArmParams FRONTARMPARAMS = new FrontArmParams();

    // Drive Motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // IMU
    private IMU imu;

    // Lift Motor
    private DcMotor lift;

    // Bucket Servo
    private Servo bucket;

    // Front Arm Motor
    private DcMotor frontArm;

    // Grabber Servos
    private Servo grabber;

    private Servo wrist;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // IMU stuff
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        // CONSTANTS
        double yawAngle;
        double Speed_percentage;

        // Drive Motors
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // FRONT ARM MOTOR
        frontArm = hardwareMap.get(DcMotor.class, "frontArm");

        // Grabber Servos
        grabber = hardwareMap.get(Servo.class, "grabber");

        wrist = hardwareMap.get(Servo.class, "wrist");

        // Lift Motors
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Bucket Servo
        bucket = hardwareMap.get(Servo.class, "bucket");

        // Motor Init shenanigans
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO CHANGE THIS MAYBE
        // Lift stuff
        double lift_speed_limit = .5;

        // TODO CHANGE THIS CRAPADAP
        double lift_minPos = -320.0; // highest point (up on robot)
        double lift_maxPos = 10.0; // lowest point (fwd on robot)

        double lift_power;

        // Wrist stuff
        double wrist_pos = -1;

        boolean auto_wrist_rotate = true;

        Speed_percentage = 0.6;
        yawAngle = 0;
        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                telemetry.addData("Yaw/Rotation Angle", "Press Circle or B on Gamepad to reset.");
                // Check to see if reset yaw is requested.
                if (gamepad1.circle || gamepad1.b)   {
                    imu.resetYaw();
                }
                orientation = imu.getRobotYawPitchRollAngles();

                yawAngle = orientation.getYaw(AngleUnit.RADIANS);

                auto_wrist_rotate = (frontArm.getCurrentPosition() < FRONTARMPARAMS.autoRangeLimPos); // don't mind the name of the constant

                // Uncomment this line when possible
                double theta = yawAngle;
                // PI / 2; = 90 degrees (in terms of radians)

                double _inputX = gamepad1.left_stick_x;
                double _inputY = gamepad1.left_stick_y;

                // Changing vectors of joystick input
                double robotInputY = ((_inputX * Math.sin(theta)) + (_inputY * Math.sin(theta + (PI / 2)))); // *1.4
                double robotInputX = (_inputX * Math.cos(theta)) + (_inputY * Math.cos(theta + (PI / 2)));


                // Robot-centric drive base code (with edits to robotInputY and robotInputX turn this into Field-centric drive)
                double rightBackPower = (robotInputY + -robotInputX + gamepad1.right_stick_x) * Speed_percentage;
                double leftBackPower = (robotInputY + robotInputX + -gamepad1.right_stick_x) * Speed_percentage;
                double rightFrontPower = (robotInputY + robotInputX + gamepad1.right_stick_x) * Speed_percentage;
                double leftFrontPower = (robotInputY + -robotInputX + -gamepad1.right_stick_x) * Speed_percentage;

                // Telemetry
                // Drive motor telemetry
                telemetry.addData("Power of leftBack", leftBack.getPower());
                telemetry.addData("Power of rightBack", rightBack.getPower());
                telemetry.addData("Power of leftFront", leftFront.getPower());
                telemetry.addData("Power of rightFront", rightFront.getPower());

                // Lift telemetry
                telemetry.addData("Power of lift", lift.getPower()); // positive power is up, negative power is down
                telemetry.addData("Position of lift", lift.getCurrentPosition()); // highest pos should be 4400 or near that

                // Grabber telemetry
                telemetry.addData("Position of grabber", grabber.getPosition());
                telemetry.addData("Position of wrist", wrist.getPosition());

                // Bucket telemetry
                telemetry.addData("Position of bucket", bucket.getPosition());

                // Front Arm telemetry
                telemetry.addData("Power of frontArm", frontArm.getPower());
                telemetry.addData("Position of frontArm", frontArm.getCurrentPosition());

                // ------------------------------ACTUAL MOVEMENT STUFF------------------------------

                // BOOSTER BUTTON!!!!!
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }

                // GRABBER
                // close grabber if either bumpers/ trigger is pressed
                if (gamepad2.left_trigger >= .5 || gamepad2.right_trigger >= .5 || gamepad2.left_bumper || gamepad2.right_bumper) {
                    grabber.setPosition(GRABBERPARAMS.closedPosition);
                    telemetry.addData("Grabber status", "closed");
                } else { // open grabber when bumpers/ triggers are not pressed
                    grabber.setPosition(GRABBERPARAMS.openPosition);
                    telemetry.addData("Grabber status", "open");
                }

                // WRIST
                if (gamepad2.a) {
                    wrist_pos = WRISTPARAMS.subPosition; // sub - A
                } else if (gamepad2.b) {
                    wrist_pos = WRISTPARAMS.groundPosition; // ground - B
                } else if (gamepad2.x) {
                    wrist_pos = WRISTPARAMS.bucketPosition; // bucket - X
                } else if (gamepad2.left_stick_y < 0 && auto_wrist_rotate) { // when front arm go up and front arm is within auto range, wrist to bucket
                    // if left stick / frontArm goes up, then flip wrist to bucket
                    wrist_pos = WRISTPARAMS.bucketPosition; // bucket
                } else if (gamepad2.left_stick_y > 0 && auto_wrist_rotate) { // when front arm go down and front arm is within auto range, wrist to ground
                    // if left stick / frontArm goes down, then flip wrist to ground
                    wrist_pos = WRISTPARAMS.groundPosition;
                }
                if (frontArm.getCurrentPosition() < FRONTARMPARAMS.autoRangeLimPos && lift.getPower() > 0) {    // if lift is going down AND front arm is folded in, flip wrist to sub
                    wrist_pos = WRISTPARAMS.subPosition;
                }

                // normalization (actually no this is beans... idk what I actually put here
                if (wrist_pos != -1) {
                    if (gamepad2.dpad_down) {
                        wrist_pos += .1;
                    } else if (gamepad2.dpad_up) {
                        wrist_pos -= .1;
                    }

                    if (wrist_pos > 1) {
                        wrist_pos = 1;
                    } else if (wrist_pos < 0) {
                        wrist_pos = 0;
                    }
                    wrist.setPosition(wrist_pos);
                }


                // BUCKET CRAP
                if (gamepad2.y) {
                    bucket.setPosition(BUCKETPARAMS.flippedPosition); // flipped backward
                } else {
                    bucket.setPosition(BUCKETPARAMS.normalPosition); // normal position
                }

                // LIFT CRAP
                // TODO Add limiter?
                lift.setPower(-gamepad2.right_stick_y); // speed limit used to be .7

                // FRONT ARM CRAP
                frontArm.setPower(gamepad2.left_stick_y * .5); // speed limit used to be .7

                // negative front arm power is up

                // FIELD CENTRIC CRAP

                /* highestPower is the highest value out of all of the absolute values of
                rightBackPower, leftBackPower, rightFrontPower, and leftFrontPower. */
                double highestPower = Math.max(Math.max(Math.abs(rightBackPower), Math.abs(leftBackPower)),
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));


                // Normalizing powers (the powers will never go above 1)
                if (highestPower > 1) {
                    leftBackPower = leftBackPower / highestPower;
                    rightBackPower = rightBackPower / highestPower;
                    leftFrontPower = leftFrontPower / highestPower;
                    rightFrontPower = rightFrontPower / highestPower;
                }

                // directional driving based on robot position
//                rightBack.setPower((gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * speedPercentage);
//                leftBack.setPower((gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * speedPercentage);
//                rightFront.setPower((gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * speedPercentage);
//                leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * speedPercentage);

                rightBack.setPower((rightBackPower));
                leftBack.setPower((leftBackPower));
                rightFront.setPower((rightFrontPower));
                leftFront.setPower((leftFrontPower));
                telemetry.update();
                // Get the orientation and angular velocity.

                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));
//                telemetry.addData("stick_y", gamepad1.left_stick_y);
//
//                telemetry.addData("robotInputY", robotInputY);
//                telemetry.addData("leftYCos", gamepad1.left_stick_y * Math.cos(theta));
//                telemetry.addData("leftXSin", gamepad1.left_stick_x * Math.sin(theta));

//                telemetry.addData("stick_x", gamepad1.left_stick_x);
//
//                telemetry.addData("robotInputX", robotInputX);
//                telemetry.addData("leftYSin", gamepad1.left_stick_y * Math.sin(theta));
//                telemetry.addData("LeftXCos", gamepad1.left_stick_x * Math.cos(theta));

//
//                telemetry.addData("rightStickY", gamepad1.right_stick_y);
//                telemetry.addData("rightStickX", gamepad1.right_stick_x);
//
//                telemetry.addData("Yaw Angle", yawAngle);
                telemetry.update();
            }
        }
    }
}