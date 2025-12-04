package org.firstinspires.ftc.teamcode.teleops.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {

    public static class Speeds {
        public double IN = 1.0;
        public double OUT = -1.0; // currently half
        public double OFF = 0.0;
    }

    public static Speeds speeds = new Speeds();

    private DcMotor intake;

    private enum IntakeState {
        INTAKING,
        OUTTAKING,
        OFF
    }

    private IntakeState intakeState;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeState = IntakeState.OFF;
    }

    public void updateIntake() {
        // could add state machine logic here if needed
        switch (intakeState) {
            case INTAKING:
                
                intake.setPower(speeds.IN);
                break;

            case OUTTAKING:
                
                intake.setPower(speeds.OUT);
                break;

            case OFF:
                
                intake.setPower(speeds.OFF);
                break;
        
            default:
                break;
        }
    }

    public void intake() {
        intakeState = IntakeState.INTAKING;
    }
    public void outtake() {
        intakeState = IntakeState.OUTTAKING;
    }
    public void stop() {
        intakeState = IntakeState.OFF;
    }
}
