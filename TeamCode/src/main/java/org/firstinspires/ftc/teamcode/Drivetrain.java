package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private IMU imu;

    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Telemetry telemetry;

    private static final double FRONT_POWER_COEFFICIENT = 0.8;

    public enum DriveMode {
        ROBOT_CENTRIC, FIELD_CENTRIC, FIELD_CENTRIC_POWER_ADJUSTED
    }

    DriveMode driveMode;

    public Drivetrain(HardwareMap hardwareMap,
                      Gamepad gamepad1,
                      Telemetry telemetry,
                      DriveMode driveMode) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
        this.driveMode = driveMode;
    }

    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("front-left");
        backLeftMotor = hardwareMap.dcMotor.get("back-left");
        frontRightMotor = hardwareMap.dcMotor.get("front-right");
        backRightMotor = hardwareMap.dcMotor.get("back-right");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);
    }

    public void loop() {
        if(driveMode == DriveMode.ROBOT_CENTRIC){
            robotCentricLoop();
        } else if(driveMode == DriveMode.FIELD_CENTRIC) {
            fieldCentricLoop();
        } else if(driveMode == DriveMode.FIELD_CENTRIC_POWER_ADJUSTED){
            fieldCentricPowerAdjustedLoop();
        }
    }

    private void robotCentricLoop(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing

        double rx = gamepad1.left_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void fieldCentricLoop() {
        // Reset IMU yaw when the options button is pressed
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Get joystick inputs (y is negated due to inverted y-axis)
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafe with slight multiplier
        double rx = gamepad1.right_stick_x;  // Rotation

        // Get robot heading from IMU in radians
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate joystick inputs for field-centric control
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers with normalization
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Update telemetry for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addData("Heading", botHeading);
        telemetry.update();
    }

    private void fieldCentricPowerAdjustedLoop() {
        // Reset IMU yaw when the options button is pressed
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Get joystick inputs (y is negated due to inverted y-axis)
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafe with slight multiplier
        double rx = gamepad1.right_stick_x;  // Rotation

        // Get robot heading from IMU in radians
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate joystick inputs for field-centric control
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate base motor powers
        double frontLeftBasePower = rotY + rotX + rx;
        double backLeftBasePower = rotY - rotX + rx;
        double frontRightBasePower = rotY - rotX - rx;
        double backRightBasePower = rotY + rotX - rx;

        // Apply power adjustment for front wheels due to rear-shifted center of mass
        double frontLeftPower = frontLeftBasePower;
        double frontRightPower = frontRightBasePower;
        double backLeftPower = backLeftBasePower * FRONT_POWER_COEFFICIENT;  // Back wheels retain full power
        double backRightPower = backRightBasePower * FRONT_POWER_COEFFICIENT;

        // Normalize powers to ensure they stay within [-1, 1]
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        double denominator = Math.max(maxPower, 1);  // Use max power instead of sum for simplicity
        frontLeftPower /= denominator;
        backLeftPower /= denominator;
        frontRightPower /= denominator;
        backRightPower /= denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Update telemetry for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addData("Heading", botHeading);
        telemetry.addData("Front Power Coefficient", FRONT_POWER_COEFFICIENT);
        telemetry.update();
    }
}