package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClaw {
    private Servo clawServo;
    private Servo rotationServo;
    private HardwareMap hardwareMap;
    private Gamepad gamepad2;
    private Telemetry telemetry;

    // Claw constants
    private static final double CLOSED_POSITION = 0.0;
    private static final double OPEN_POSITION = 0.158;
    private boolean isOpen = false;
    private boolean isTransitioning = false;
    private boolean lastR2State = false;

    // Rotation constants
    private static final double MIN_ROTATION = 0.5;
    private static final double MAX_ROTATION = 0.75;
    private static final double ROTATION_SPEED = 0.003;
    private double targetRotation = MIN_ROTATION;
    private double currentRotation = MIN_ROTATION;

    public IntakeClaw(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        clawServo = hardwareMap.servo.get("intake-claw-grab");
        rotationServo = hardwareMap.servo.get("intake-claw-z-rotation");

        clawServo.setPosition(CLOSED_POSITION);
        rotationServo.setPosition(MIN_ROTATION);

        isOpen = false;
        isTransitioning = false;
        currentRotation = MIN_ROTATION;
        targetRotation = MIN_ROTATION;
    }

    public void loop() {
        // Claw control logic
        boolean currentR2State = gamepad2.right_trigger > 0.5;
        if (currentR2State && !lastR2State && !isTransitioning) {
            isTransitioning = true;
            if (isOpen) {
                clawServo.setPosition(CLOSED_POSITION);
                isOpen = false;
            } else {
                clawServo.setPosition(OPEN_POSITION);
                isOpen = true;
            }
            new java.util.Timer().schedule(
                    new java.util.TimerTask() {
                        @Override
                        public void run() {
                            isTransitioning = false;
                        }
                    },
                    200
            );
        }
        lastR2State = currentR2State;

        // Rotation control logic
        double rightStickX = gamepad2.right_stick_x;

        // Increment or decrement currentRotation only while stick is held beyond threshold
        if (rightStickX > 0.85) {
            currentRotation = Math.min(currentRotation + ROTATION_SPEED, MAX_ROTATION);
        } else if (rightStickX < -0.85) {
            currentRotation = Math.max(currentRotation - ROTATION_SPEED, MIN_ROTATION);
        }
        // No else clause needed; servo holds position when stick is neutral

        rotationServo.setPosition(currentRotation);

        // Telemetry data
        telemetry.addData("Claw Position", clawServo.getPosition());
        telemetry.addData("Is Open", isOpen);
        telemetry.addData("Rotation Position", rotationServo.getPosition());
        telemetry.addData("Right Stick X", gamepad2.right_stick_x);
        telemetry.update();
    }
}