package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private DcMotor viperLeftMotor;
//    private DcMotor viperRightMotor;

    HardwareMap hardwareMap;
    Gamepad gamepad2;
    Telemetry telemetry;

    // Position variables for each state (in encoder ticks)
    private int basePosition = 0;
    private int highChamber1Position = 1000; // A little lower than mid height
    private int highChamber2Position = 1500; // Mid height
    private int fullyExtendedPosition = 2160; // Full height

    private States currentState = States.BASE;

    // Variables to detect button press rising edges
    private boolean prevDpadDown = false;
    private boolean prevDpadUp = false;

    public Lift(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad1; // Assuming gamepad1 is intended for operator controls
        this.telemetry = telemetry;
    }

    enum States {
        BASE,           // 0 height
        HIGH_CHAMBER_1, // A little lower than mid height
        HIGH_CHAMBER_2, // Mid height
        FULLY_EXTENDED  // Full height
    }

    public void init() {
        viperLeftMotor = hardwareMap.dcMotor.get("viper-left");
//        viperRightMotor = hardwareMap.dcMotor.get("viper-right");

        viperLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        viperRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        viperRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        viperRightMotor.setTargetPosition(0);
//        viperRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperLeftMotor.setTargetPosition(0);
        viperLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set initial power to 0 to prevent movement until commanded
        viperLeftMotor.setPower(0);
//        viperRightMotor.setPower(0);
    }

    public void loop() {
        // Read current gamepad states
        boolean dpadDown = gamepad2.dpad_down;
        boolean dpadUp = gamepad2.dpad_up;
        boolean leftBumper = gamepad2.left_bumper;

        // Detect rising edges for button presses
        boolean dpadDownPressed = dpadDown && !prevDpadDown;
        boolean dpadUpPressed = dpadUp && !prevDpadUp;

        // Update previous states for next loop
        prevDpadDown = dpadDown;
        prevDpadUp = dpadUp;

        // Only process inputs if motors are not transitioning
        if (!viperLeftMotor.isBusy()
//                &&
//                !viperRightMotor.isBusy()
        ) {
            States newState = currentState;

            // Check for special transitions with L1 (left bumper)
            if (leftBumper) {
                if (dpadDownPressed) {
                    newState = States.BASE;
                } else if (dpadUpPressed) {
                    newState = States.FULLY_EXTENDED;
                }
            }
            // Check for normal state transitions
            else {
                if (dpadDownPressed && currentState.ordinal() > 0) {
                    newState = States.values()[currentState.ordinal() - 1];
                } else if (dpadUpPressed && currentState.ordinal() < 3) {
                    newState = States.values()[currentState.ordinal() + 1];
                }
            }

            // If the state has changed, update motor targets
            if (newState != currentState) {
                currentState = newState;
                int targetPosition = getPositionForState(currentState);
                viperLeftMotor.setTargetPosition(targetPosition);
//                viperRightMotor.setTargetPosition(targetPosition);
                viperLeftMotor.setPower(1.0);
//                viperRightMotor.setPower(1.0);
            }
        }
    }

    // Helper method to map states to their respective positions
    private int getPositionForState(States state) {
        double degreesToTicks = 537.7 / 360;
        int targetAngleInDegrees;
        switch (state) {
            case BASE:
                targetAngleInDegrees = basePosition;
                break;
            case HIGH_CHAMBER_1:
                targetAngleInDegrees = highChamber1Position;
                break;
            case HIGH_CHAMBER_2:
                targetAngleInDegrees = highChamber2Position;
                break;
            case FULLY_EXTENDED:
                targetAngleInDegrees = fullyExtendedPosition;
                break;
            default:
                targetAngleInDegrees = 0;
                break;// Fallback to base position
        }
        return (int)(targetAngleInDegrees * degreesToTicks);
    }
}