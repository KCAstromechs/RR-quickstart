package org.firstinspires.ftc.teamcode.teleops.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;




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

    private DcMotor frontRight, frontLeft, backRight, backLeft;

    private double speedPercentage, yawAngle;

    private enum BoostState {
        BOOSTING,
        NORMAL
    }

    private BoostState boostState;

    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        // TODO edit directions

        // To allow automatic braking, set 'zero power behavor' to brake for all motors
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

    public void updateDrive() {
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
    }

    public void boost() {
        boostState = BoostState.BOOSTING;
    }

    public void normal() {
        boostState = BoostState.NORMAL;
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

    // TODO add telemetry methods
}
