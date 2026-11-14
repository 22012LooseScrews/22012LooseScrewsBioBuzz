package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.PollenSect.PollenServo;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous
public class RedParkAuto extends LinearOpMode {
    DcMotor rf, lf, rr, lr;
    OuttakeMotor outtake_motor;
    IntakeMotor intake_motor;
    PollenServo pollen_servo;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(62.5, 133.5, 270);
    private final Pose shoot1 = poseFactory.of(29.3181, 119.1431, 317.5);
    private final Pose park_start = poseFactory.of(29.3181, 119.1431, 317.5);
    private final Pose park = poseFactory.of(10.7012, 121.3834, 0);
    private final Pose park_control = poseFactory.of(73.9801, 120.2168, 0);

//    Command auto = sequential(
//            follow(follower, shoot1()),
//            instant(() -> outtake_motor.outtake_close()),
//            waitMs(7500),
//            instant(() -> outtake_motor.outtake_stop()),
//            follow(follower, park())
//    );
    public Command shootfar() {
        return sequential(
                instant(() -> outtake_motor.outtake_far()),
                instant(() -> pollen_servo.pollenServo_up()),
                waitMs(7500),
                instant(() -> intake_motor.intake_intake()),
                waitMs(2000),
                instant(() -> outtake_motor.outtake_stop()),
                instant(()-> intake_motor.intake_stop())
        );
    }
    public Command auto() {
        return sequential(
                //shoot1
                follow(follower, shoot1()),
                shootfar(),

                //intake1
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(auto());

        while(opModeIsActive()){
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path park() {
        return Paths.curve(park_start, park_control, park).linear(park_start, park);
    }
}
