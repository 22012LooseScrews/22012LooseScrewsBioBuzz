package org.firstinspires.ftc.teamcode.abstractions.LimelightSect;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class LimelightServo {
    private Servo limelightServo;
    public LimelightServo(OpMode opMode){
        limelightServo = opMode.hardwareMap.get(Servo.class, "limelightServo");
        limelightServo.setDirection(Servo.Direction.FORWARD);
    }
    public void limelightServo_up() {
        limelightServo.setPosition(0.25);
    }
    public void limelightServo_down() {
        limelightServo.setPosition(0.75);
    }
    public void limelightServo_zero() {
        limelightServo.setPosition(0);
    }
    public void limelightServo_one() {
        limelightServo.setPosition(1);
    }
}
