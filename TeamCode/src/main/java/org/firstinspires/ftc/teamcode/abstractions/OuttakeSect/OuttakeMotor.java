package org.firstinspires.ftc.teamcode.abstractions.OuttakeSect;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OuttakeMotor {
    private DcMotorEx outtakeMotor;
    private DcMotorEx outtakeMotor2;
//    private final double auto_close_rpm = 1575;
    private final double close_rpm = 1600;
    private final double far_rpm = 1750;

    public OuttakeMotor(OpMode opMode) {
        outtakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setVelocityPIDFCoefficients(
                70,
                0,
                0,
                11.0
        );
        outtakeMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor2");
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor2.setVelocityPIDFCoefficients(
                70,
                0,
                0,
                11.0
        );
    }

//    public void auto_outtake_close() {
//        outtakeMotor.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                12.365
//        );
//        outtakeMotor.setVelocity(-auto_close_rpm);
//    }
    public void outtake_close() {
//        outtakeMotor.setVelocityPIDFCoefficients(
//                80,
//                0,
//                0,
//                13.0
//        );
//        outtakeMotor.setVelocity(-close_rpm);
        outtakeMotor.setVelocity(close_rpm);
        outtakeMotor2.setVelocity(close_rpm);
    }
    public void outtake_far() {
         outtakeMotor.setVelocityPIDFCoefficients(
                70,
                0,
                0,
                11
        );
//        outtakeMotor.setVelocity(-far_rpm);
        outtakeMotor.setVelocity(far_rpm);
        outtakeMotor2.setVelocity(far_rpm);
    }
    public void outtake_stop() {
        outtakeMotor.setVelocity(0);
        outtakeMotor2.setVelocity(0);
    }
    public double getVel(){
        return outtakeMotor.getVelocity();
    }
    public void outtake_spec(float x) {
        outtakeMotor.setPower(x);
    }
}
