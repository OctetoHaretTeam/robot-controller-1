package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class TeleOpV3_FieldCentric extends OpMode {

    Drivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, Drivetrain.DriveMode.FIELD_CENTRIC_POWER_ADJUSTED);
        drivetrain.init();
    }

    /**
     * This is called repeatedly while OpMode is playing
     */
    @Override
    public void loop() {
        drivetrain.loop();
    }
}