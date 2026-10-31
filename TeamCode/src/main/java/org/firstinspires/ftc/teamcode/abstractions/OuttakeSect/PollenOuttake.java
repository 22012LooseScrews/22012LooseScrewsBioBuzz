package org.firstinspires.ftc.teamcode.abstractions.OuttakeSect;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PollenOuttake {
    private DcMotorEx pollenMotor;

    private DcMotorEx pollenMotor2;
    private double auto_close_rpm = 1575;
    private double close_rpm = 1575;
    private double far_rpm = 1960;

    public PollenOuttake(OpMode opMode) {
        pollenMotor = opMode.hardwareMap.get(DcMotorEx.class, "pollenMotor");
        pollenMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        pollenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        pollenMotor.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
    }

    public void auto_outtake_close() {
        pollenMotor.setVelocityPIDFCoefficients(
                80,
                0,
                0,
                12.365
        );
        pollenMotor.setVelocity(-auto_close_rpm);
    }
    public void outtake_close() {
//        pollenMotor.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
//        pollenMotor.setVelocity(-close_rpm);
        pollenMotor.setPower(0.75);
    }
    public void outtake_far() {
//        pollenMotor.setVelocityPIDFCoefficients(
//                90,
//                0,
//                0,
//                12.4
//        );
//        pollenMotor.setVelocity(-far_rpm);
        pollenMotor.setPower(1.0);
    }
    public void outtake_stop() {
        pollenMotor.setPower(0);
    }
    public double getVel(){
        return pollenMotor.getVelocity();
    }
    public void outtake_spec(float x) {
        pollenMotor.setPower(x);
    }
}
