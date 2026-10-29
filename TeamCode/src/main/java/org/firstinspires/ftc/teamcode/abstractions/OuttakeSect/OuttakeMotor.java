package org.firstinspires.ftc.teamcode.abstractions.OuttakeSect;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OuttakeMotor {
    private DcMotorEx outtakeMotor;

    private DcMotorEx outtakeMotor2;
    private double auto_close_rpm = 1575;
    private double close_rpm = 1575;
    private double far_rpm = 1960;

    public OuttakeMotor(OpMode opMode) {
        outtakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtakeMotor.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
    }

    public void auto_outtake_close() {
        outtakeMotor.setVelocityPIDFCoefficients(
                80,
                0,
                0,
                12.365
        );
        outtakeMotor.setVelocity(-auto_close_rpm);
    }
    public void outtake_close() {
//        outtakeMotor.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
//        outtakeMotor.setVelocity(-close_rpm);
        outtakeMotor.setPower(0.75);
    }
    public void outtake_far() {
//        outtakeMotor.setVelocityPIDFCoefficients(
//                90,
//                0,
//                0,
//                12.4
//        );
//        outtakeMotor.setVelocity(-far_rpm);
        outtakeMotor.setPower(1.0);
    }
    public void outtake_stop() {
        outtakeMotor.setPower(0);
    }
    public double getVel(){
        return outtakeMotor.getVelocity();
    }
    public void outtake_spec(float x) {
        outtakeMotor.setPower(x);
    }
}
