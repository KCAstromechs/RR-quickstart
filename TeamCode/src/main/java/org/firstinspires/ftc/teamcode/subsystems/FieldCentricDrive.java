package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class FieldCentricDrive {

    public static class Speeds {
        public double normalSpeed = 0.6;
        public double boostSpeed = 1.0;

        //other?
    }
    public static Speeds speeds = new Speeds();

    private IMU imu;
    private YawPitchRollAngles orientation;
    private double speedPercentage, yawAngle;

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    /*
    Ports:
    - FR: 2
    - FL: 0
    - BR: 3
    - BL: 1
     */

    public enum BoostState {
        BOOSTING,
        NORMAL
    }

    private BoostState boostState;

    public enum AutoAimState {
        ON,
        OFF
    }
    private AutoAimState autoAimState;

    // auto aim vars
    double kP = 0.0020;
    double error = 0;
    double lastError = 0;
    double tgtAngle = 0; // offset here
    double angleTolerance = 0.5;

    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        // TODO edit directions

        // To allow automatic braking, set 'zero power behavior' to brake for all motors
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boostState = BoostState.NORMAL;
        speedPercentage = speeds.normalSpeed;
        yawAngle = 0;

        // Initialize the IMU.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        // TODO edit the orientation of IMU
    }

    public void updateYaw() {
        orientation = imu.getRobotYawPitchRollAngles();
        yawAngle = orientation.getYaw(AngleUnit.RADIANS);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    /**
     * Updates the drive subsystem
     *  - Updates yaw / rotation
     *  - updates boost state
     */
    public void updateDrive(AprilTagDetection aprilTagDetection, OpMode opMode, double moveX, double moveY, double rotX) {
        // update yaw
        updateYaw();

        // update boostState
        switch (boostState) {
            case BOOSTING:
                speedPercentage = speeds.boostSpeed;
                break;
            case NORMAL:
                speedPercentage = speeds.normalSpeed;
                break;
        }

        // auto aim logic
        switch (autoAimState) {
            case ON:
                if (aprilTagDetection != null) {
                    error = tgtAngle - aprilTagDetection.ftcPose.bearing; // tx (error = where you want to be MINUS where you are)

                    if (Math.abs(error) < angleTolerance) {
                        rotX = 0;
                    } else {
                        double pTerm = error * kP;
                        curTime = opMode.getRuntime();
                        double dT = curTime - lastTime; // change in time
                        double dTerm = ((error - lastError) / dT) * kD;

                        rotX = Range.clip(pTerm + dTerm, -0.4, 0.4);

                        lastError = error;
                        lastTime = curTime;
                    }
                } else {
                    lastError = 0;
                    lastTime = opMode.getRuntime();
                }
            case OFF:
                lastError = 0;
                lastTime = opMode.getRuntime();
        }

        // drive motors
        drive(moveX, moveY, rotX);
    }

    public void boost() {
        if (boostState == BoostState.NORMAL) boostState = BoostState.BOOSTING;
    }

    public void normal() {
        if (boostState == BoostState.BOOSTING) boostState = BoostState.NORMAL;
    }

    public void enableAutoAim() {
        autoAimState = AutoAimState.ON;
    }

    public void disableAutoAim() {
        autoAimState = AutoAimState.OFF;
    }

    public void drive(double moveX, double moveY, double rotX) {
        double theta = yawAngle;
        // PI / 2; = 90 degrees (in terms of radians)

        // Changing vectors of joystick input
        double robotInputY = ((moveX * Math.sin(theta)) + (moveY * Math.sin(theta + (Math.PI / 2)))); // *1.4
        double robotInputX = (moveX * Math.cos(theta)) + (moveY * Math.cos(theta + (Math.PI / 2)));

        // Robot-centric drive base code (with edits to robotInputY and robotInputX turn this into Field-centric drive)
        double rightBackPower = (robotInputY + -robotInputX + rotX) * speedPercentage;
        double leftBackPower = (robotInputY + robotInputX + -rotX) * speedPercentage;
        double rightFrontPower = (robotInputY + robotInputX + rotX) * speedPercentage;
        double leftFrontPower = (robotInputY + -robotInputX + -rotX) * speedPercentage;

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
        frontRight.setPower((rightFrontPower));
        frontLeft.setPower((leftFrontPower));
        backRight.setPower((rightBackPower));
        backLeft.setPower((leftBackPower));
    }

    public String getState() {
        return boostState.toString();
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addLine("Drive Telemetry:");
        telemetry.addLine("Motor Powers (F = front, B = back)");
        telemetry.addData("F Motors", "left (%.2f), right (%.2f)", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("B Motors", "left (%.2f), right (%.2f)", backLeft.getPower(), backRight.getPower());
        telemetry.addLine("Other");
        telemetry.addData("Boost State", getState());
    }
}