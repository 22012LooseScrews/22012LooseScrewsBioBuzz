package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
@TeleOp
public class OuttakeTestingTeleop extends OpMode {
    DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    OuttakeMotor outtake_motor;
    @Override
    public void init(){
//        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake_motor");
        outtake_motor = new OuttakeMotor(this);
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);
        if (gamepad1.right_trigger > 0.1) {
            outtake_motor.outtake_far();
        } else if (gamepad1.left_trigger > 0.1) {
            outtake_motor.outtake_close();
        } else {
            outtake_motor.outtake_stop();
        }
    }
}
