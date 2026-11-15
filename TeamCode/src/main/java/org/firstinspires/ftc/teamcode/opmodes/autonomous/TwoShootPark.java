package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.HoodServo;
import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.PollenSect.PollenServo;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class TwoShootPark extends LinearOpMode {
    IntakeMotor intake_motor;
    OuttakeMotor outtake_motor;
    PollenServo pollen_servo;
    HoodServo hood_servo;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(38.4, 10, 90);
    private final Pose path1 = poseFactory.of(29.3, 22.4, 42.5);
    private final Pose point2Start = poseFactory.of(29.3, 22.4, 42.5);
    private final Pose point2 = poseFactory.of(17.6, 54, 180);
    private final Pose point3Start = poseFactory.of(17.6, 54, 180);
    private final Pose point3 = poseFactory.of(14.7, 54, 180);
    private final Pose jerk1start = poseFactory.of(14.75, 54, 180);
    private final Pose jerk1 = poseFactory.of(22.5, 54, 180);
    private final Pose jerk2start = poseFactory.of(22.5, 54, 180);
    private final Pose jerk2 = poseFactory.of(15.5, 54, 180);
    private final Pose point4Start = poseFactory.of(15.5, 54, 180);
    private final Pose point4 = poseFactory.of(29.3, 22.4, 42.5);
    private final Pose point5Start = poseFactory.of(29.3, 22.4, 42.5);
    private final Pose point5 = poseFactory.of(12.4, 118, 180);
    private final Pose point6Start = poseFactory.of(15.5, 54, 180);
    private final Pose point6 = poseFactory.of(29.3, 22.4, 42.5);
    private final Pose point7Start = poseFactory.of(29.3, 22.4, 42.5);
    private final Pose point7 = poseFactory.of(12.4726, 118.2557, 180);

    public Command shootfar() {
        return sequential(
                instant(() -> hood_servo.hoodServo_up()),
                instant(() -> outtake_motor.outtake_far()),
                instant(() -> pollen_servo.pollenServo_up()),
                waitMs(2500),
                instant(() -> intake_motor.intake_intake()),
                waitMs(2000),
                instant(() -> outtake_motor.outtake_stop())
        );
    }

    public Command intakeflower1() {
        return sequential(
                instant(() -> hood_servo.hoodServo_down()),
                instant(() -> pollen_servo.pollenServo_down()),
                instant(() -> intake_motor.intake_intake()),
                follow(follower, path3()),
                waitMs(3000),
                instant(() -> intake_motor.intake_stop()),
                follow(follower, jerk1()),
                follow(follower, jerk2())

        );
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                //shoot1
                follow(follower, path1()),
                shootfar(),
                instant(() -> hood_servo.hoodServo_down()),

                //intake1
                follow(follower, path2()),
                intakeflower1(),

                //shoot2
                instant(() -> hood_servo.hoodServo_up()),
                follow(follower, path4()),
                shootfar(),
                follow(follower, path5())


        );
    }

    @Override
    public void runOpMode() {
        intake_motor = new IntakeMotor(this);
        outtake_motor = new OuttakeMotor(this);
        pollen_servo = new PollenServo(this);
        hood_servo = new HoodServo(this);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

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

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }
    public Path jerk1() {return Paths.line(jerk1start,jerk1).linear(jerk1start,jerk1);}
    public Path jerk2() {return Paths.line(jerk2start,jerk2).linear(jerk2start,jerk2);}

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }
    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }
}