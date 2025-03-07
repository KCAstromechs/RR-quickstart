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
@Autonomous(name = "AutoLeftNoRR (Java)", preselectTeleOp = "TELELEOPTETESTING (Java)")
public class AutoLeftNoRR extends LinearOpMode {

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
    double speed = -0.5; // this is negative bc bucket is front now

    // Yaw (for rotate)
    double yawAngle;

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
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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
            WRIST_TO_SUB();
            RESET_BUCKET();
            OPEN_GRABBER();
            sleep(200);
            MOVE_FORWARD(590);
            sleep(100);
            STRAFE_RIGHT(100);
            sleep(50);
//            OPEN_GRABBER();
            RAISE_LIFT();
//            sleep(10);
            FLIP_BUCKET();
            sleep(800);
            RESET_BUCKET();
            sleep(100);
            LOWER_LIFT();
//            CLOSE_GRABBER();
            sleep(10);
            // At this point, we have set the pre-loaded sample into the high basket.
            WRIST_TO_GROUND();
            MOVE_BACKWARD(200);
            STRAFE_RIGHT(400);
            TURN_LEFT(88);
            GRAB_SAMPLE(100);
            TURN_RIGHT(60); // need to fine tune this angle
            MOVE_FORWARD(450); // need to tune this distance
            RAISE_LIFT(); // Raise 2nd sample
            FLIP_BUCKET(); // slam dunk
            sleep(800);
            RESET_BUCKET(); // move bucket away
            sleep(100);
            LOWER_LIFT(); // bring the lift back down
            TURN_LEFT(54);
            WRIST_TO_GROUND();
            GRAB_SAMPLE(230);
            MOVE_FORWARD(230);
            TURN_RIGHT(56);
            RAISE_LIFT(); // Raise 3rd sample
            FLIP_BUCKET(); // slam dunk
            sleep(800);
//            RESET_BUCKET(); // move bucket away
//            sleep(100);
//            LOWER_LIFT(); // bring the lift back down



//            MOVE_FORWARD(300);
//            TURN_RIGHT(90);
//            STRAFE_LEFT(300); // Strafe to line up with wall
//            MOVE_FORWARD(250); // Move forward to reach the high basket
//            RAISE_LIFT(); // Raise 2nd sample
//            FLIP_BUCKET(); // slam dunk
//            sleep(1000);
//            RESET_BUCKET(); // move bucket away
//            sleep(100);
//            //LOWER_LIFT(); // bring the lift back down
//            // At this point, approximately 30 seconds have passed
        }
    }


    /**
     * Grabs a sample
     */
    private void GRAB_SAMPLE(int dist_away_from_sample) {
        LOWER_ARM();
        MOVE_BACKWARD(dist_away_from_sample); // TO GRAB SAMPLE ON GROUND
        CLOSE_GRABBER();
        sleep(200);
        RAISE_ARM();
        WRIST_TO_BUCKET();
        sleep(400);
        OPEN_GRABBER();
        sleep(100);
        WRIST_TO_SUB();
    }

    /**
     * STOP the robot
     */
    private void STOP_ROBOT() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        sleep(200);
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
     * @param degrees degrees to turn in degrees
     */
    private void TURN_RIGHT(int degrees) {
        imu.resetYaw();
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
        while (Math.abs(yawAngle) < degrees) {
            yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("yawAngle", yawAngle);
            telemetry.update();
        }
        STOP_ROBOT();
    }


    /**
     * Turn left certain # of encoder clicks (in terms of front_left)
     * @param degrees distance to turn in degrees
     */
    private void TURN_LEFT(int degrees) {
        imu.resetYaw();
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
        while (Math.abs(yawAngle) < degrees) {
            yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("yawAngle", yawAngle);
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
        sleep(100);
    }


    /**
     * Close grabber
     */
    private void CLOSE_GRABBER() {
        grabber.setPosition(0.9);
        telemetry.addData("Grabber status", "closed");
        telemetry.update();
        sleep(100);
    }

    private void WRIST_TO_GROUND() {
        wrist.setPosition(.75);
    }
    private void WRIST_TO_SUB() {
        wrist.setPosition(.9);
    }
    private void WRIST_TO_BUCKET() {
        wrist.setPosition(.55);
    }


    /**
     * Moves lift to 'low' position
     */
    private void LOWER_LIFT() {
        if (lift.getCurrentPosition() > 50) {
            while (lift.getCurrentPosition() > 50) {
                lift.setPower(-0.5);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        } else if (lift.getCurrentPosition() < 50) {
            while (lift.getCurrentPosition() < 50) {
                lift.setPower(0.5);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        }
        sleep(200);
    }

    /**
     * Moves lift to 'highest' position
     */
    private void RAISE_LIFT() {
        if (lift.getCurrentPosition() > 4000) {
            while (lift.getCurrentPosition() > 4000) {
                lift.setPower(-1);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        } else if (lift.getCurrentPosition() < 4000) {
            while (lift.getCurrentPosition() < 4000) {
                lift.setPower(1);
                LIFT_UPDATE(lift.getCurrentPosition());
            }
            lift.setPower(0);
        }
        sleep(200);
    }

    /**
     * Raises lift to a custom pos
     * @param custom_pos the position the lift will raise to (in encoder clicks)
     */
    private void RAISE_LIFT_CUSTOM(int custom_pos) {
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
        sleep(200);
    }

    /**
     * Puts lift pos to telemetry
     * @param pos this should be lift.getCurrentPosition() in some form
     */
    private void LIFT_UPDATE(double pos) {
        telemetry.addData("Lift Actual Pos: ", pos);
        telemetry.update();
    }

    /**
     * Puts lift pos to telemetry
     * @param pos this should be lift.getCurrentPosition() in some form
     */
    private void FRONT_ARM_UPDATE(double pos) {
        telemetry.addData("Front Arm Actual Pos: ", pos);
        telemetry.update();
    }



    private void FLIP_BUCKET() {

        bucket.setPosition(0);
        telemetry.addData("Bucket Status: ", "flipped");
        telemetry.update();
        sleep(200);
    }

    private void RESET_BUCKET() {
        bucket.setPosition(0.7);
        telemetry.addData("Bucket Status: ", "normal");
        telemetry.update();
        sleep(200);
    }


// TODO TEST THESE NUMBERS
    // NEGATIVE POWER IS UP FOR FRONT ARM
    // POSITIVE POWER IS DOWN FOR FRONT ARM
    // ~400 encoder clicks is arm down
    // ~0 encoder clicks is arm up (I use 50 instead of 0 so we don't bonk ourself
    private void RAISE_ARM() {
        if (frontArm.getCurrentPosition() > 50) {
            while (frontArm.getCurrentPosition() > 50) {
                frontArm.setPower(-0.6);
                FRONT_ARM_UPDATE(frontArm.getCurrentPosition());
            }
            frontArm.setPower(0);
        } else if (frontArm.getCurrentPosition() < 50) {
            while (frontArm.getCurrentPosition() < 50) {
                frontArm.setPower(0.6);
                FRONT_ARM_UPDATE(frontArm.getCurrentPosition());
            }
            frontArm.setPower(0);
        }
        sleep(200);
    }

    private void LOWER_ARM() {
        if (frontArm.getCurrentPosition() > 275) {
            while (frontArm.getCurrentPosition() > 275) {
                frontArm.setPower(-0.6);
                FRONT_ARM_UPDATE(frontArm.getCurrentPosition());
            }
            frontArm.setPower(0);
        } else if (frontArm.getCurrentPosition() < 250) {
            while (frontArm.getCurrentPosition() < 250) {
                frontArm.setPower(0.6);
                FRONT_ARM_UPDATE(frontArm.getCurrentPosition());
            }
            frontArm.setPower(0);
        }
        sleep(500);
    }
}