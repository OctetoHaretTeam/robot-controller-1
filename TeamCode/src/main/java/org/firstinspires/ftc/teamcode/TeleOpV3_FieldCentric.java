package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class TeleOpV3_FieldCentric extends OpMode {

    Drivetrain drivetrain;
    Lift lift;
    IntakeClaw intakeClaw;

    IntakeArm intakeArm;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, Drivetrain.DriveMode.FIELD_CENTRIC_POWER_ADJUSTED);
        drivetrain.init();

        lift = new Lift(hardwareMap, gamepad2, telemetry);
        lift.init();

        intakeClaw = new IntakeClaw(hardwareMap, gamepad2, telemetry);
        intakeClaw.init();

        intakeArm = new IntakeArm(hardwareMap, gamepad2, telemetry);
        intakeArm.init();
    }

    /**
     * This is called repeatedly while OpMode is playing
     */
    @Override
    public void loop() {
        drivetrain.loop();
        lift.loop();
        intakeClaw.loop();
        intakeArm.loop();
    }
}