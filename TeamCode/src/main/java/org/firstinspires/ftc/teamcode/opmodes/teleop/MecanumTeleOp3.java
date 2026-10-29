package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class MecanumTeleOp3 extends OpMode {
    DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    IMU imu;
    @Override
    public void init() {
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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revOrientation));
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x * 1.1;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double theta = Math.atan2(x, y);
        double r = Math.hypot(y, x);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double new_y = r * Math.sin(theta);
        double new_x = r * Math.cos(theta);

        double denominator = Math.max(Math.abs(new_y) + Math.abs(new_x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((new_y + new_x + rx) / denominator);
        backLeftMotor.setPower((new_y - new_x + rx) / denominator);
        frontRightMotor.setPower((new_y - new_x - rx) / denominator);
        backRightMotor.setPower((new_y + new_x - rx) / denominator);
    }
}
