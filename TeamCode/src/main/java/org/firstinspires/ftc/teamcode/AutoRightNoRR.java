// THIS IS MARK 3 CODE (Field Centric)

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "AutoRightNoRR (Java)", preselectTeleOp = "TELELEOPTETESTING (Java)")
public class AutoRightNoRR extends LinearOpMode {

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

    // Sped
    private double speed = -0.4;

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
            MOVE_FORWARD(2000);
            

        }
    }
    /**
     * STOP the robot
     */
    private void STOP_ROBOT() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

    /**
     * Move forward certain distance
     * @param distanceEncoders distance to travel in terms of encoders
     */
    private void MOVE_FORWARD(int distanceEncoders) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        while (Math.abs(leftFront.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Move backward certain distance (in terms of front_left motor encoders)
     * @param distanceEncoders distance to travel in terms of encoders
     */
    private void MOVE_BACKWARD(int distanceEncoders) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(-speed);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(-speed);
        while (Math.abs(leftFront.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Strafe right certain distance (in terms of front_left motor encoders)
     * @param distanceEncoders distance to travel in terms of encoders
     */
    private void STRAFE_RIGHT(int distanceEncoders) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
        while (Math.abs(leftFront.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Strafe left certain distance (in terms of front_left motor encoders)
     * @param distanceEncoders distance to travel in terms of encoders
     */
    private void STRAFE_LEFT(int distanceEncoders) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
        while (Math.abs(leftFront.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Turn right certain # of encoder clicks (in terms of front_left)
     * @param distanceEncoders degrees to turn in terms of encoders
     */
    private void TURN_RIGHT(int distanceEncoders) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
        while (Math.abs(leftFront.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }


    /**
     * Turn left certain # of encoder clicks (in terms of front_left)
     * @param distanceEncoders distance to turn in terms of encoders
     */
    private void TURN_LEFT(int distanceEncoders) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
        while (Math.abs(leftFront.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }


    /**
     * Open grabber
     */
    private void OPEN_GRABBER() {
        grabber.setPosition(0.65);
        telemetry.addData("Grabber status", "open");
        telemetry.update();
        sleep(1000);
    }


    /**
     * Close grabber
     */
    private void CLOSE_GRABBER() {
        grabber.setPosition(0.9);
        telemetry.addData("Grabber status", "closed");
        telemetry.update();
        sleep(1000);
    }


    /**
     * Moves lift to 'low' position
     */
    private void LOWER_LIFT() {
        sleep(200);
        if (lift.getCurrentPosition() > 100) {
            while (lift.getCurrentPosition() > 100) {
                lift.setPower(-0.3);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        } else if (lift.getCurrentPosition() < 100) {
            while (lift.getCurrentPosition() < 100) {
                lift.setPower(0.3);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        }
    }

    /**
     * Moves lift to 'highest' position
     */
    private void RAISE_LIFT() {
        sleep(200);
        if (lift.getCurrentPosition() > 4000) {
            while (lift.getCurrentPosition() > 4000) {
                lift.setPower(-0.5);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        } else if (lift.getCurrentPosition() < 4000) {
            while (lift.getCurrentPosition() < 4000) {
                lift.setPower(0.5);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        }
    }

    /**
     * Raises lift to a custom pos
     * @param custom_pos the position the lift will raise to (in encoder clicks)
     */
    private void RAISE_LIFT_CUSTOM(int custom_pos) {
        sleep(200);
        if (lift.getCurrentPosition() > custom_pos) {
            while (lift.getCurrentPosition() > custom_pos) {
                lift.setPower(-0.5);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        } else if (lift.getCurrentPosition() < custom_pos) {
            while (lift.getCurrentPosition() < custom_pos) {
                lift.setPower(0.5);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        }
    }

    /**
     * Puts lift pos to telemetry
     * @param pos this should be lift.getCurrentPosition() in some form
     */
    private void LIFT_UPDATE(double pos) {
        telemetry.addData("Lift Actual Pos: ", pos);
        telemetry.update();
    }

    private void FLIP_BUCKET() {

        bucket.setPosition(0);
        telemetry.addData("Bucket Status: ", "flipped");
        telemetry.update();
        sleep(1000);
    }

    private void RESET_BUCKET() {
        bucket.setPosition(0.7);
        telemetry.addData("Bucket Status: ", "normal");
        telemetry.update();
        sleep(1000);
    }
}