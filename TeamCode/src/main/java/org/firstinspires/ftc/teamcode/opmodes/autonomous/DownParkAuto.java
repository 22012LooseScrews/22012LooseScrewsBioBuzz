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

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.PollenSect.PollenServo;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous
public class DownParkAuto extends LinearOpMode {
    OuttakeMotor outtake_motor;
    IntakeMotor intake_motor;
    PollenServo pollen_servo;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(110, 8, 90);
    private final Pose shoot1 = poseFactory.of(112.18, 22.3569, 132.5);
    private final Pose park_start = poseFactory.of(112.18, 22.3569, 132.5);
    private final Pose park = poseFactory.of(132.3957, 22.4232, 0);

    public Command shootfar() {
        return sequential(
                waitMs(5000),
                instant(() -> outtake_motor.outtake_far()),
                instant(() -> pollen_servo.pollenServo_up()),
                waitMs(3000),
                instant(() -> intake_motor.intake_intake()),
                waitMs(2000),
                instant(() -> outtake_motor.outtake_stop()),
                instant(()-> intake_motor.intake_stop())
        );
    }

    public Command auto() {
        return sequential(
            follow(follower, shoot1()),
            instant(() ->outtake_motor.outtake_close()),
            waitMs(2500),
            instant(() ->outtake_motor.outtake_stop()),
            follow(follower, park())
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
        return Paths.line(park_start, park).linear(park_start, park);
    }
}
