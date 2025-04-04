package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class ViperTestOp extends OpMode {

    DcMotor viperRightMotor;

    @Override
    public void init() {
        viperRightMotor = hardwareMap.dcMotor.get("viper-right");
        viperRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        viperRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperRightMotor.setTargetPosition(0);
        viperRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            double degreesToTicks = 537.7 / 360;
            double targetAngle = 1080;
            int targetPosition = (int) (targetAngle * degreesToTicks);

            viperRightMotor.setPower(1);
            viperRightMotor.setTargetPosition(targetPosition);
            telemetry.addData("Motor Position:", viperRightMotor.getCurrentPosition());
        }
    }
}
