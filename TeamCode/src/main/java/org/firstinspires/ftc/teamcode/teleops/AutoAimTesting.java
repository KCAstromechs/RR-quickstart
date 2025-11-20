package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "AutoAimTesting", group = "Testing")
public class AutoAimTesting extends OpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // IMU
    private IMU imu;

    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;

    // IMU stuff
    private YawPitchRollAngles orientation;
    private AngularVelocity angularVelocity;

    // CONSTANTS
    private double yawAngle;
    private double Speed_percentage;

    private double turnMultiplier = 0;
    private final double turnMultiplierMax = 2;
    private double angleOfDeflectionTolerance = 2;

    @Override
    public void init() {

        aprilTagWebcam.init(hardwareMap, telemetry);
        // IMU stuff
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        // CONSTANTS
        double yawAngle;
        double Speed_percentage;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRight  = hardwareMap.get(DcMotor.class, "rightFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        // To allow automatic braking, set 'zero power behavor' to brake for all motors
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Speed_percentage = 0.6;
        yawAngle = 0;
        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        // Prompt user to press start button.
        telemetry.addData("Initialization finished", "Press start to continue...");
//        telemetry.addData("exposure (i think plz idk man)", aprilTagWebcam.getExposureControl().getExposure(TimeUnit.MILLISECONDS));
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Yaw/Rotation Angle", "Press Y on Gamepad to reset.");
        // Check to see if reset yaw is requested.
        if (gamepad1.y)   {
            imu.resetYaw();
        }
        orientation = imu.getRobotYawPitchRollAngles();

        yawAngle = orientation.getYaw(AngleUnit.RADIANS);

        // BOOSTER BUTTON!!!!!
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            Speed_percentage = 1;
        } else {
            Speed_percentage = 0.6;
        }
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
        backRight.setPower((rightBackPower));
        backLeft.setPower((leftBackPower));
        frontRight.setPower((rightFrontPower));
        frontLeft.setPower((leftFrontPower));
        // Get the orientation and angular velocity.

        // update the vison portal
        aprilTagWebcam.update();
        AprilTagDetection idRed = aprilTagWebcam.getTagBySpecificId(24);
        AprilTagDetection idBlue = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemtry(idRed);
        aprilTagWebcam.displayDetectionTelemtry(idBlue);
//        aprilTagWebcam.stop();

        // auto aim
        /*
        if b (angle of deflection) > 0, turn left (slowly)
        else if b (angle of deflection < 0, turn right (slowly)
        else (if no tags detected, add "no tags detected" to telemetry)
        (for all cases, add a multiplier to the turning speed based on how far off center the tag is)
         */
        if (gamepad1.a) {
            if (idRed != null) {
                //            double angleOfDeflection = idRed.xAngleToTag;
                double angleOfDeflection = idRed.ftcPose.bearing;
                telemetry.addData("Red Angle of Deflection", angleOfDeflection);
                double turnMultiplier = Math.min(turnMultiplierMax, Math.max(Math.abs(angleOfDeflection) / 30, 0));
                if (angleOfDeflection > angleOfDeflectionTolerance) {
                    //          turnMultiplier = JavaUtil.clamp(angleOfDeflection / 30, 0, turnMultiplierMax);
                    backRight.setPower(-Speed_percentage * turnMultiplier);
                    backLeft.setPower(Speed_percentage * turnMultiplier);
                    frontRight.setPower(-Speed_percentage * turnMultiplier);
                    frontLeft.setPower(Speed_percentage * turnMultiplier);
                } else if (angleOfDeflection < -angleOfDeflectionTolerance) {
                    //                  turnMultiplier = JavaUtil.clamp(-angleOfDeflection / 30, 0, turnMultiplierMax);
                    backRight.setPower(Speed_percentage * turnMultiplier);
                    backLeft.setPower(-Speed_percentage * turnMultiplier);
                    frontRight.setPower(Speed_percentage * turnMultiplier);
                    frontLeft.setPower(-Speed_percentage * turnMultiplier);
                } else {
                    // stop turning
                    backRight.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    frontLeft.setPower(0);
                }
            } else if (idBlue != null) {
                //            double angleOfDeflection = idBlue.xAngleToTag;
                double angleOfDeflection = idBlue.ftcPose.bearing;
                telemetry.addData("Blue Angle of Deflection", angleOfDeflection);
                turnMultiplier = Math.min(turnMultiplierMax, Math.max(Math.abs(angleOfDeflection) / 30, 0));
                if (angleOfDeflection > angleOfDeflectionTolerance) {
                    //                turnMultiplier = JavaUtil.clamp(angleOfDeflection / 30, 0, turnMultiplierMax);
                    backRight.setPower(-Speed_percentage * turnMultiplier);
                    backLeft.setPower(Speed_percentage * turnMultiplier);
                    frontRight.setPower(-Speed_percentage * turnMultiplier);
                    frontLeft.setPower(Speed_percentage * turnMultiplier);
                } else if (angleOfDeflection < -angleOfDeflectionTolerance) {
                    //                turnMultiplier = JavaUtil.clamp(-angleOfDeflection / 30, 0, turnMultiplierMax);
                    backRight.setPower(Speed_percentage * turnMultiplier);
                    backLeft.setPower(-Speed_percentage * turnMultiplier);
                    frontRight.setPower(Speed_percentage * turnMultiplier);
                    frontLeft.setPower(-Speed_percentage * turnMultiplier);
                } else {
                    // stop turning
                    backRight.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    frontLeft.setPower(0);
                }
            } else {
                telemetry.addData("Tag Detection", "No Tags Detected");
            }
        }

        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));
        telemetry.addData("Encoder pos of leftFront", frontLeft.getCurrentPosition());
        telemetry.update();
    }
}
