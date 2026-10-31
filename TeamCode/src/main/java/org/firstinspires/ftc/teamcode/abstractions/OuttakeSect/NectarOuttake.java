package org.firstinspires.ftc.teamcode.abstractions.OuttakeSect;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class NectarOuttake {
    private DcMotorEx nectarMotor;

    private double auto_close_rpm = 1575;
    private double close_rpm = 1575;
    private double far_rpm = 1960;

    public NectarOuttake(OpMode opMode) {
        nectarMotor = opMode.hardwareMap.get(DcMotorEx.class, "nectarouttake");
        nectarMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        nectarouttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        nectarouttake.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
    }

    public void auto_outtake_close() {
        nectarMotor.setVelocityPIDFCoefficients(
                80,
                0,
                0,
                12.365
        );
        nectarMotor.setVelocity(-auto_close_rpm);
    }
    public void outtake_close() {
//        nectarouttake.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
//        nectarouttake.setVelocity(-close_rpm);
        nectarMotor.setPower(0.75);
    }
    public void outtake_far() {
//        nectarouttake.setVelocityPIDFCoefficients(
//                90,
//                0,
//                0,
//                12.4
//        );
//        nectarouttake.setVelocity(-far_rpm);
        nectarMotor.setPower(1.0);
    }
    public void outtake_stop() {
        nectarMotor.setPower(0);
    }
    public double getVel(){
        return nectarMotor.getVelocity();
    }
    public void outtake_spec(float x) {
        nectarMotor.setPower(x);
    }
}
