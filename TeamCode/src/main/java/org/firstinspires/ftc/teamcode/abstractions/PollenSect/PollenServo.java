package org.firstinspires.ftc.teamcode.abstractions.PollenSect;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class PollenServo {
    private Servo pollenServo;
    public PollenServo(OpMode opMode){
        pollenServo = opMode.hardwareMap.get(Servo.class, "pollenServo");
        pollenServo.setDirection(Servo.Direction.FORWARD);
    }
    public void pollenServo_up() {
        pollenServo.setPosition(0.5);
    }
    public void pollenServo_down() {
        pollenServo.setPosition(0.75);
    }
    public void pollenServo_zero() {
        pollenServo.setPosition(0);
    }
    public void pollenServo_one() {
        pollenServo.setPosition(1);
    }
}

