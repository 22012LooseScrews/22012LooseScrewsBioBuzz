package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous
public class RedBottomBasicAuto extends LinearOpMode {
    IntakeMotor intakeMotor = new IntakeMotor(this);
    OuttakeMotor outtakeMotor = new OuttakeMotor(this);
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose balls = poseFactory.of(23.2, 8.1, 180);
    private final Pose balls_control = poseFactory.of(62.0674, 19.9378, 0);
    private final Pose intake1_start = poseFactory.of(23.2, 8.1, 180);
    private final Pose intake1 = poseFactory.of(8.6, 8.1, 180);
    private final Pose shoot1_start = poseFactory.of(8.6, 8.1, 180);
    private final Pose shoot1 = poseFactory.of(32.4, 128, 305);
    private final Pose shoot1_control1 = poseFactory.of(41.8333, 32.7288, 0);
    private final Pose shoot1_control2 = poseFactory.of(19.0104, 108.5285, 0);
    private final Pose park_start = poseFactory.of(32.4, 128, 305);
    private final Pose park = poseFactory.of(11.592, 120.375, 300);

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
    public void runOpMode() throws InterruptedException{
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
