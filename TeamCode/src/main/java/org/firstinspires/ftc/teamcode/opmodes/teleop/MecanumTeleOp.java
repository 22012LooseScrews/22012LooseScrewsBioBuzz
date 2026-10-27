package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.LimelightSect.LimelightServo;

// Red Alliance Tags - 30-37
// Blue Alliance Tags - 38-45
@TeleOp
public class MecanumTeleOp extends OpMode {
    DcMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    IntakeMotor intakeMotor1;
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
        intakeMotor1 = new IntakeMotor(this);
    }

    @Override
    public void loop(){
        double x = -gamepad1.left_stick_x * 1.1;
        double y = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((y + x + rx) / denominator);
        backLeftMotor.setPower((y - x + rx) / denominator);
        frontRightMotor.setPower((y - x - rx) / denominator);
        backRightMotor.setPower((y + x - rx) / denominator);

        if(gamepad1.left_bumper){
            intakeMotor1.intake_intake();
        }
        else if(gamepad1.right_bumper){
            intakeMotor1.intake_outtake();
        }
        else{
            intakeMotor1.intake_stop();
        }
    }
}
