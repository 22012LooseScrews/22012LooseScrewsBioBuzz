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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class RedParkAuto extends LinearOpMode {
    OuttakeMotor outtakeMotor = new OuttakeMotor(this);
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(62.5, 133.5, 270);
    private final Pose shoot1 = poseFactory.of(58.1473, 128.5123, 270);
    private final Pose park_start = poseFactory.of(58.1473, 128.5123, 270);
    private final Pose park = poseFactory.of(10.7012, 121.3834, 0);
    private final Pose park_control = poseFactory.of(73.9801, 120.2168, 0);

    Command auto = sequential(
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
