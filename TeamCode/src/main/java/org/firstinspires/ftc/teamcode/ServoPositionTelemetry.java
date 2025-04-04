package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Position Telemetry")
public class ServoPositionTelemetry extends OpMode {
    private Servo servo;
    private double targetPosition = 0.0;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "intake-arm-y-angle");
        servo.setPosition(0.0); // Start at minimum position
    }

    @Override
    public void loop() {
        // Update target position
        if (gamepad2.square) {
            targetPosition += 0.001; // Increment position
            if (targetPosition >= 1.0) {
                targetPosition = 1.0;
            }
        } else if(gamepad2.circle) {
            targetPosition -= 0.001; // Decrement position
            if (targetPosition <= 0.0) {
                targetPosition = 0.0;
            }
        }

        // Set servo position
        servo.setPosition(targetPosition);

        // Display current position in telemetry
        telemetry.addData("Intake Claw Grab Position", servo.getPosition());
    }
}