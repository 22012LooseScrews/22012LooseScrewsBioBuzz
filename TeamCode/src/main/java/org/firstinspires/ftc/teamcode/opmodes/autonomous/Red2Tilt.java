package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.IntakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.IntakeSect.HoodServo;
import org.firstinspires.ftc.teamcode.abstractions.OuttakeSect.OuttakeMotor;
import org.firstinspires.ftc.teamcode.abstractions.LimelightSect.LimelightServo;
import org.firstinspires.ftc.teamcode.abstractions.PollenSect.PollenServo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class Red2Tilt extends LinearOpMode {
    DcMotor rf, lf, rr, lr;
    IntakeMotor intake_motor;
    OuttakeMotor outtake_motor;
    PollenServo pollen_servo;
    HoodServo hood_servo;
    LimelightServo limelight_servo;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(38.4022, 10.0164, 90);
    private final Pose path1 = poseFactory.of(29.3181, 22.3569, 42.5);
    private final Pose point2Start = poseFactory.of(29.3181, 22.3569, 42.5);
    private final Pose point2 = poseFactory.of(17.6069, 50.8038, 180);
    private final Pose point3Start = poseFactory.of(17.6069, 50.8038, 180);
    private final Pose point3 = poseFactory.of(14.75, 50.8038, 180);
    private final Pose jerk1start = poseFactory.of(14.75, 50.8038, 180);
    private final Pose jerk1 = poseFactory.of(22.5, 50.8038, 180);
    private final Pose jerk2start = poseFactory.of(22.5, 50.8038, 180);
    private final Pose jerk2 = poseFactory.of(15.5, 50.8038, 180);
    private final Pose point4Start = poseFactory.of(15.5, 50.8038, 180);
    private final Pose point4 = poseFactory.of(30.681, 122.5143, 300);
    private final Pose point4Control1 = poseFactory.of(36.7662, 67.4089, 0);
    private final Pose point5Start = poseFactory.of(30.681, 122.5143, 300);
    private final Pose point5 = poseFactory.of(46.5916, 124.4168, 90);
    private final Pose point6Start = poseFactory.of(46.5916, 124.4168, 90);
    private final Pose point6 = poseFactory.of(46.5223, 128.9492, 90);
    private final Pose point7Start = poseFactory.of(46.5223, 128.9492, 90);
    private final Pose point7 = poseFactory.of(30.8285, 122.1844, 302);
    private final Pose point7Control1 = poseFactory.of(42.808, 114.4799, 0);
    private final Pose point8 = poseFactory.of(13.9338, 116.1762, 180);

    // Autonomous routine
    public Command shootfar() {
        return sequential(
            instant(() -> outtake_motor.outtake_far()),
            instant(() -> pollen_servo.pollenServo_up()),
            waitMs(3000),
            instant(() -> intake_motor.intake_intake()),
            waitMs(2000),
            instant(() -> outtake_motor.outtake_stop()),
            instant(()-> intake_motor.intake_stop())
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
    public Command intakeflower2() {
        return sequential(
            instant(() -> intake_motor.intake_intake()),
            instant(() -> pollen_servo.pollenServo_down()),
            follow(follower, path6()),
            waitMs(3000),
            instant(() -> intake_motor.intake_stop())
        );
    }
    public Command otherside() {
        return parallel(
            follow(follower,path4()),
            instant(() -> hood_servo.hoodServo_up())
        );
    }
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
                follow(follower, path4()),

                shootfar(),

                //intake2
                follow(follower, path5()),
                intakeflower2(),

                //shoot3
                follow(follower, path7()),
                shootfar(),

                //park
                follow(follower, path8())
        );
    }

    @Override
    public void runOpMode() {
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
        pollen_servo = new PollenServo(this);
        hood_servo = new HoodServo(this);
        limelight_servo = new LimelightServo(this);
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
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).linear(point7, point8);
    }
}
