package org.firstinspires.ftc.teamcode.teleops;

// import org.firstinspires.ftc.teamcode.teleops.subsystems.Intake;
// import org.firstinspires.ftc.teamcode.teleops.subsystems.FieldCentricDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MainTeleOp extends OpMode {

    private FieldCentricDrive drive = new FieldCentricDrive();
    private Intake intake = new Intake();

    @Override
    public void init() {
        
        drive.init(hardwareMap);
        intake.init(hardwareMap);


    }

    @Override
    public void loop() {



        // update subsystems
        drive.updateDrive();
        intake.updateIntake();
    }
}
