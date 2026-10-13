package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
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
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous
public class BlueTopBasicAuto extends LinearOpMode {
    IntakeMotor intakeMotor = new IntakeMotor(this);
    OuttakeMotor outtakeMotor = new OuttakeMotor(this);
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(85.5, 133.5, 270);
    private final Pose balls = poseFactory.of(118.3, 133.4, 0);
    private final Pose balls_control = poseFactory.of(79.4326, 121.5622, 0);
    private final Pose intake1_start = poseFactory.of(118.3, 133.4, 0);
    private final Pose intake1 = poseFactory.of(132.9, 133.4, 0);
    private final Pose shoot1_start = poseFactory.of(132.9, 133.4, 0);
    private final Pose shoot1 = poseFactory.of(109.1, 13.5, 125);
    private final Pose shoot1_control1 = poseFactory.of(99.6667, 108.7712, 0);
    private final Pose shoot1_control2 = poseFactory.of(122.4896, 32.9715, 0);
    private final Pose park_start = poseFactory.of(109.1, 13.5, 125);
    private final Pose park = poseFactory.of(129.908, 21.125, 120);

    Command auto = sequential(
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

    public Path balls(){
        return Paths.curve(start, balls_control, balls).linear(start, balls);
    }
    public Path intake1(){
        return Paths.line(intake1_start, intake1).linear(intake1_start, intake1);
    }
    public Path shoot1(){
        return Paths.curve(shoot1_start, shoot1_control1, shoot1_control2, shoot1).linear(shoot1_start, shoot1);
    }
    public Path park(){
        return Paths.line(park_start, park).linear(park_start, park);
    }
}
