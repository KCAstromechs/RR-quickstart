package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    public static class Params {
        public double IN = 1.0;
        public double RE_IN = 0.5;
        public double OUT = -1.0; // currently half
        public double OFF = 0.0;
    }

    public static Params params = new Params();

    private DcMotor intake;

    public enum IntakeState {
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

    public void updateIntake(LauncherPID.LauncherState launcherState) {
        switch (launcherState) {
            case OFF:
                switch (intakeState) {
                    case INTAKING:

                        intake.setPower(params.IN);
                        break;

                    case OUTTAKING:

                        intake.setPower(params.OUT);
                        break;

                    case OFF:

                        intake.setPower(params.OFF);
                        break;

                    default:
                        break;
                }
                break;
            case SPIN_UP:
                intake.setPower(params.OFF);
                break;
            case LAUNCHING:
                intake.setPower(params.IN);
                break;
            case RESPIN_UP:
                intake.setPower(params.RE_IN);
            default:
                break;
        }
    }

    public void intake() {
        if (intakeState != IntakeState.INTAKING) intakeState = IntakeState.INTAKING;
    }
    public void outtake() {
        if (intakeState != IntakeState.OUTTAKING) intakeState = IntakeState.OUTTAKING;
    }
    public void stop() {
        if (intakeState != IntakeState.OFF) intakeState = IntakeState.OFF;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addLine("Intake Telemetry:");
        telemetry.addData("Intake State", getIntakeState());
    }
}