package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.LimelightSect.LimelightServo;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.PollenSect.PollenServo;

@TeleOp
public class FieldCentricMecanumTeleop extends LinearOpMode {
    IntakeMotor intake_motor;
    OuttakeMotor outtake_motor;
    PollenServo pollen_servo;
    LimelightServo limelight_servo;
    boolean pservoup = false;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        intake_motor = new IntakeMotor(this);
        outtake_motor = new OuttakeMotor(this);
        pollen_servo = new PollenServo(this);
        limelight_servo = new LimelightServo(this);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.left_bumper){
                intake_motor.intake_intake();
            }
            else if(gamepad1.right_bumper){
                intake_motor.intake_outtake();
            }
            else{
                intake_motor.intake_stop();
            }

            if (gamepad1.left_trigger > 0.1) {
                outtake_motor.outtake_close();
            } else if (gamepad1.right_trigger > 0.1) {
                outtake_motor.outtake_far();
            } else {
                outtake_motor.outtake_stop();
            }

            if (gamepad1.dpad_up && pservoup) {
                pollen_servo.pollenServo_down();
                pservoup = false;
            }
            else if(gamepad1.dpad_up) {
                pollen_servo.pollenServo_up();
                pservoup = true;
            }

            if(gamepad1.dpad_left){
                pollen_servo.pollenServo_down();
            }
            else if(gamepad1.dpad_right){
                pollen_servo.pollenServo_up();
            }
        }
    }
}
