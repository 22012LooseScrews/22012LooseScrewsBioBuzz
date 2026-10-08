package org.firstinspires.ftc.teamcode.abstractions.IntakeSect;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeMotor {
    private DcMotor intakeMotor;
    public IntakeMotor(OpMode opMode){
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake_intake(){
        intakeMotor.setPower(1);
    }
    public void intake_outtake(){
        intakeMotor.setPower(-1);
    }
    public void intake_stop(){
        intakeMotor.setPower(0);
    }
    public void intake_spec_in(double n) {intakeMotor.setPower((n+0.3)%1.01);}
    public void intake_spec_out(double n) {intakeMotor.setPower(-((n+0.3)%1.01));}
}
