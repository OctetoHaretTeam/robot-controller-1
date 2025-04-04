package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Position Telemetry")
public class MotorPositionTelemetry extends OpMode {
    private DcMotor motor;
    private int targetPosition = 0;

    private boolean prevDpadDown = false;
    private boolean prevDpadUp = false;

    private int currentPosition = 0;

    @Override
    public void init() {
        // Initialize the motor from the hardware map
        motor = hardwareMap.get(DcMotor.class, "intake-arm");
        // Set motor to brake when power is zero
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset encoder to zero
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set motor to run to position mode
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(0.6);
    }

    @Override
    public void loop() {
        motor.setPower(0.3);
        if(!motor.isBusy() && gamepad2.square){
            currentPosition += 10;
            motor.setTargetPosition(currentPosition);
        }
        if(!motor.isBusy() && gamepad2.circle){
            currentPosition -= 10;
            currentPosition = Math.max(currentPosition, 0);
            motor.setTargetPosition(currentPosition);
        }
        telemetry.addData("Motor Rotation:", currentPosition);
    }
}