package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class ViperTestOp extends OpMode {

    DcMotor viperRightMotor;
    DcMotor viperLeftMotor;

    @Override
    public void init() {
//        viperRightMotor = hardwareMap.dcMotor.get("viper-right");
        viperLeftMotor = hardwareMap.dcMotor.get("viper-left");
//        viperRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        viperRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        viperRightMotor.setTargetPosition(0);
//        viperRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperLeftMotor.setTargetPosition(0);
        viperLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            double degreesToTicks = 537.7 / 360;
            double targetAngle = 90;
            int targetPosition = (int) (targetAngle * degreesToTicks);

//            viperRightMotor.setPower(0.7);
            viperLeftMotor.setPower(0.7);
//            viperRightMotor.setTargetPosition(targetPosition);
            viperLeftMotor.setTargetPosition(targetPosition);
            telemetry.addData("Motor Position:", viperRightMotor.getCurrentPosition());
        }
    }
}
