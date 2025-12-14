package org.firstinspires.ftc.teamcode.teleops;

// the homemade subsystems
import org.firstinspires.ftc.teamcode.teleops.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.teleops.subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleops.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.teleops.subsystems.Spindexer;

// other other imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTeleOp", group = "Comp")
public class MainTeleOp extends OpMode {

    private FieldCentricDrive drive = new FieldCentricDrive();
    private Intake intake = new Intake();
    private Launcher launcher = new Launcher();
    private Spindexer spindexer = new Spindexer(); // TODO finish spindexer subsystem

    @Override
    public void init() {

        drive.init(hardwareMap);
        intake.init(hardwareMap);
//        launcher.init(hardwareMap);
//        spindexer.init(hardwareMap);

    }

    @Override
    public void loop() {

        // Attachment Keybinds
        // intake
        if (Math.abs(gamepad1.right_trigger) > .25 || Math.abs(gamepad1.left_trigger) > .25) {
            intake.intake();
        } else if (gamepad1.dpad_down) {
            intake.outtake();
        } else {
            intake.stop();
        }

        // TODO launcher
//        if (Math.abs(gamepad2.left_trigger) > 0.25 || Math.abs(gamepad2.right_trigger) > 0.25) {
//            launcher.startLauncher();
//        } else if (gamepad2.b) {
//            launcher.stopLauncher();
//        }

        // TODO spindexer ? - may not need keybinds, just an update method with more params

        // Drive Keybinds
        drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x); // movement
        // boost
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            drive.boost();
        } else {
            drive.normal();
        }

        // update subsystems
        drive.updateDrive();
        intake.updateIntake();
//        launcher.updateLauncher();
//        spindexer.updateSpindexer();


        // show telemetry
        intake.displayTelemetry(telemetry);
        drive.displayTelemetry(telemetry);
    }
}
