package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class RedMostWorkAuto extends LinearOpMode {
    IntakeMotor intakeMotor = new IntakeMotor(this);
    OuttakeMotor outtakeMotor = new OuttakeMotor(this);
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(61, 133.5, 270);
    private final Pose rotate = poseFactory.of(58.1473, 12.9877, 90);
    private final Pose rotate_control = poseFactory.of(61.4413, 62.506, 0);
    private final Pose balls = poseFactory.of(23.2, 8.1, 180);
    private final Pose balls_control = poseFactory.of(62.0674, 19.9378, 0);
    private final Pose intake1 = poseFactory.of(8.6, 8.1, 180);
    private final Pose shoot1 = poseFactory.of(58.1473, 128.5123, 270);
    private final Pose shoot1_control = poseFactory.of(25.0501, 110.2064, 0);
    private final Pose park = poseFactory.of(10.7012, 121.3834, 180);
    private final Pose park_control = poseFactory.of(73.9801, 120.2168, 0);

    Command auto = sequential(
            follow(follower, rotate()),
            instant(() -> outtakeMotor.outtake_close()),
            waitMs(2500),
            instant(() -> outtakeMotor.outtake_stop()),
            follow(follower, balls()),
            parallel(
                    follow(follower, intake1()),
                    instant(() -> intakeMotor.intake_intake())
            ),
            instant(() -> intakeMotor.intake_stop()),
            follow(follower, shoot1()),
            instant(() -> outtakeMotor.outtake_close()),
            waitMs(2500),
            instant(() -> outtakeMotor.outtake_stop()),
            follow(follower, park())
    );

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(auto);

        while (opModeIsActive()) {
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

    public Path rotate() {
        return curve(start, rotate_control, rotate).linear(start, rotate);
    }
    public Path balls() {
        return curve(rotate, balls_control, balls).linear(rotate, balls);
    }
    public Path intake1() {
        return line(balls, intake1).linear(balls, intake1);
    }
    public Path shoot1() {
        return curve(intake1, shoot1_control, shoot1).linear(intake1, shoot1);
    }
    public Path park() {
        return curve(shoot1, park_control, park).linear(shoot1, park);
    }
}
