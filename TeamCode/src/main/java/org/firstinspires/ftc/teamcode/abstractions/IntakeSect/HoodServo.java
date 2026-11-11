package org.firstinspires.ftc.teamcode.abstractions.IntakeSect;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodServo {
    private Servo hoodServo;
    public HoodServo(OpMode opMode){
        hoodServo = opMode.hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setDirection(Servo.Direction.FORWARD);
    }
    public void hoodServo_up() {
        hoodServo.setPosition(0.5);
    }
    public void hoodServo_down() {
        hoodServo.setPosition(0.85);
    }
    public void hoodServo_zero() {
        hoodServo.setPosition(0);
    }
    public void hoodServo_one() {
        hoodServo.setPosition(1);
    }
}

