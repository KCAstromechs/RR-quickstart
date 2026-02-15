package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Progression {
    public static class Params {
        public double stopperStopPos = 0.7;
        public double stopperRaisedPos = 0.3;

        public double FWD = 1.0;
        public double BWD = -1.0;
        public double OFF = 0.0;
    }
    public static Params params = new Params();
    // params

    // motors
    private Servo stopper = null;
    private DcMotor progression = null;

    // state TODO might not need progression state
    public enum ProgressionState {
        MANUAL,
        AUTO
    }
    private ProgressionState progressionState;

    public void init(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(Servo.class, "stopper");
        progression = hardwareMap.get(DcMotor.class, "progression");
        progression.setDirection(DcMotor.Direction.REVERSE);
        progressionState = ProgressionState.AUTO;
    }

    /**
     * Updates progression based on launcherState
     * @param launcherState the state enum of a launcher obj
     */
    public void updateProgression(LauncherPID.LauncherState launcherState) {
        switch (progressionState) {
            case AUTO:
                switch (launcherState) {
                    case SPIN_UP:
                        stopper.setPosition(params.stopperRaisedPos);
                        break;
                    case LAUNCHING:
                        stopper.setPosition(params.stopperRaisedPos);
                        progression.setPower(1);
                        break;
                    case OFF:
                        stopper.setPosition(params.stopperStopPos);
                        progression.setPower(0);
                        break;
                    default:
                        break;
                }
                break;
            case MANUAL:
                break; // do nothing except allow progression to be manual
        }
    }

    // TODO manual methods to be implemented

    public ProgressionState getProgressionState() {
        return progressionState;
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addLine("Progression telemetry:");

    }
}
