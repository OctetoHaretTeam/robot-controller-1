package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArm {

    private Servo intakeArmServo;
    private DcMotor intakeArmMotor;
    HardwareMap hardwareMap;
    private Gamepad gamepad2;
    private Telemetry telemetry;

    private States currentState = States.BASE;
    private MotorStates currentMotorState = MotorStates.BASE;

    // Variables to detect button press rising edges for servo
    private boolean prevX = false;
    private boolean prevB = false;
    // Variables to detect button press rising edges for motor
    private boolean prevA = false;
    private boolean prevY = false;

    // Motor position variables
    private int basePosition = 0;
    private int passingPosition = 240;
    private int collectingPosition = 510;

    public IntakeArm(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    enum States {
        BASE,       // Servo position 0.6
        COLLECTION, // Servo position 1.0
        PASSING     // Servo position 0.25
    }

    enum MotorStates {
        BASE,       // Motor position 0
        COLLECTING,  // Motor position 510
        PASSING,    // Motor position 250

    }

    public void init() {
        intakeArmServo = hardwareMap.servo.get("intake-arm-y-angle");
        intakeArmServo.setPosition(0.6); // Initial position for BASE

        intakeArmMotor = hardwareMap.dcMotor.get("intake-arm");
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setTargetPosition(basePosition);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setPower(0.4); // Initially, no power
    }

    public void loop() {
        // Servo control logic
        boolean x = gamepad2.square; // Square button
        boolean b = gamepad2.circle; // Circle button
        boolean xPressed = x && !prevX;
        boolean bPressed = b && !prevB;
        prevX = x;
        prevB = b;

        if (xPressed) {
            if (currentState == States.PASSING) {
                currentState = States.BASE;
            } else if (currentState != States.PASSING) {
                currentState = States.values()[currentState.ordinal() + 1];
            }
        } else if (bPressed && currentState != States.BASE) {
            currentState = States.values()[currentState.ordinal() - 1];
        }

        double servoPosition = getPositionForState(currentState);
        intakeArmServo.setPosition(servoPosition);

        // Motor control logic
        boolean a = gamepad2.square; // Cross button
        boolean y = gamepad2.circle; // Triangle button
        boolean aPressed = a && !prevA;
        boolean yPressed = y && !prevY;
        prevA = a;
        prevY = y;

        if (!intakeArmMotor.isBusy()) {
            if (aPressed) {
                // Move to next motor state
                int nextOrdinal = (currentMotorState.ordinal() + 1) % MotorStates.values().length;
                currentMotorState = MotorStates.values()[nextOrdinal];
                int targetPosition = getPositionForMotorState(currentMotorState);
                intakeArmMotor.setTargetPosition(targetPosition);
            } else if (yPressed) {
                // Move to previous motor state
                int prevOrdinal = (currentMotorState.ordinal() - 1 + MotorStates.values().length) % MotorStates.values().length;
                currentMotorState = MotorStates.values()[prevOrdinal];
                int targetPosition = getPositionForMotorState(currentMotorState);
                intakeArmMotor.setTargetPosition(targetPosition);
            }
        }
    }

    // Helper method for servo positions
    private double getPositionForState(States state) {
        switch (state) {
            case BASE:
                return 0.6;
            case COLLECTION:
                return 1.0;
            case PASSING:
                return 0.25;
            default:
                return 0.6;
        }
    }

    // Helper method for motor positions
    private int getPositionForMotorState(MotorStates state) {
        switch (state) {
            case BASE:
                return basePosition;
            case PASSING:
                return passingPosition;
            case COLLECTING:
                return collectingPosition;
            default:
                return basePosition;
        }
    }
}