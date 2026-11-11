package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.HoodServo;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.LimelightSect.LimelightServo;
import org.firstinspires.ftc.teamcode.abstractions.PollenSect.PollenServo;

// Red Alliance Tags - 30-37
// Blue Alliance Tags - 38-45
@TeleOp
public class MecanumTeleOp extends OpMode {
    DcMotor rf, lf, rr, lr;
    IntakeMotor intake_motor;
    OuttakeMotor outtake_motor;
    PollenServo pollen_servo;
    HoodServo hood_servo;
    LimelightServo limelight_servo;
    boolean pservoup = false;
    boolean hoodup = false;

    @Override
    public void init() {
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lr = hardwareMap.get(DcMotor.class, "lr");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor = new IntakeMotor(this);
        outtake_motor = new OuttakeMotor(this);
        pollen_servo = new PollenServo(this);
        hood_servo = new HoodServo(this);
        limelight_servo = new LimelightServo(this);
    }

    @Override
    public void loop(){
        double x = gamepad1.left_stick_x * 1.1;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lf.setPower((y + x + rx) / denominator);
        lr.setPower((y - x + rx) / denominator);
        rf.setPower((y - x - rx) / denominator);
        rr.setPower((y + x - rx) / denominator);

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

//        if (gamepad1.dpad_up && pservoup) {
//            pollen_servo.pollenServo_down();
//            pservoup = false;
//        }
//        else if(gamepad1.dpad_up) {
//            pollen_servo.pollenServo_up();
//            pservoup = true;
//        }
//
//        if(gamepad1.dpad_left){
//            pollen_servo.pollenServo_down();
//        }
//        else if(gamepad1.dpad_right){
//            pollen_servo.pollenServo_up();
//        }

//        if (gamepad1.dpadUpWasPressed() && !pservoup) {
//            pollen_servo.pollenServo_up();
//            pservoup = true;
//        }
//        else if(gamepad1.dpadUpWasPressed() && pservoup){
//            pollen_servo.pollenServo_down();
//            pservoup = false;
//        }
//
//        if (gamepad1.dpadLeftWasPressed() && !hoodup) {
//            hood_servo.hoodServo_up();
//            hoodup = true;
//        }
//        else if(gamepad1.dpadLeftWasPressed() && hoodup) {
//            hood_servo.hoodServo_down();
//            hoodup = false;
//        }
//        if(gamepad1.dpad_down){
//            pollen_servo.pollenServo_down();
//        }
//        else if(gamepad1.dpad_up){
//            pollen_servo.pollenServo_up();
//        }
        if(gamepad1.cross){
            limelight_servo.limelightServo_up();
        }
        else if(gamepad1.triangle){
            limelight_servo.limelightServo_down();
        }

        if (gamepad1.dpad_left) {
            hood_servo.hoodServo_up();
//                pservoup = false;
        }
        else if(gamepad1.dpad_right) {
            hood_servo.hoodServo_down();
//                pservoup = true;
        }

        if(gamepad1.dpad_down){
            pollen_servo.pollenServo_down();
        }
        else if(gamepad1.dpad_up){
            pollen_servo.pollenServo_up();
        }

//        telemetry.addData("Servo: ", pservoup);
//        telemetry.addData("Hood: ", hoodup);
    }
}
