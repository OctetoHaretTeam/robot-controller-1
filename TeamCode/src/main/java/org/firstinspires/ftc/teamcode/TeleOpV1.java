package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class TeleOpV1 extends OpMode {

    Drivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, Drivetrain.DriveMode.ROBOT_CENTRIC);
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