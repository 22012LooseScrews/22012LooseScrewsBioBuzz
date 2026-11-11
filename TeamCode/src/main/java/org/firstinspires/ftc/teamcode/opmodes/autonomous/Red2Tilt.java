package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
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

    private final Pose start = poseFactory.of(31.4022, 8.0164, 90);
    private final Pose path1 = poseFactory.of(31.3181, 17.3569, 55);
    private final Pose point2Start = poseFactory.of(31.3181, 17.3569, 55);
    private final Pose point2 = poseFactory.of(17.6069, 46.8038, 180);
    private final Pose point3Start = poseFactory.of(17.6069, 46.8038, 180);
    private final Pose point3 = poseFactory.of(10.4403, 46.8, 180);
    private final Pose point4Start = poseFactory.of(10.4403, 46.8, 180);
    private final Pose point4 = poseFactory.of(28.681, 124.5143, 305);
    private final Pose point4Control1 = poseFactory.of(36.7662, 67.4089, 0);
    private final Pose point5Start = poseFactory.of(28.681, 124.5143, 305);
    private final Pose point5 = poseFactory.of(46.5916, 124.4168, 90);
    private final Pose point6Start = poseFactory.of(46.5916, 124.4168, 90);
    private final Pose point6 = poseFactory.of(46.5223, 130.9492, 90);
    private final Pose point7Start = poseFactory.of(46.5223, 130.9492, 90);
    private final Pose point7 = poseFactory.of(28.8285, 124.1844, 305);
    private final Pose point7Control1 = poseFactory.of(42.808, 114.4799, 0);
    private final Pose point8 = poseFactory.of(13.9338, 116.1762, 28.2648);

    // Autonomous routine
    public Command shootfar() {
        return sequential(
            instant(() -> outtake_motor.outtake_close()),
            waitMs(2000),
            instant(() -> intake_motor.intake_intake()),
            waitMs(2500),
            instant(() -> outtake_motor.outtake_stop()),
            instant(()-> intake_motor.intake_stop())
        );
    }
    public Command intakeflower1() {
        return sequential(
            instant(() -> hood_servo.hoodServo_down()),
            instant(() -> intake_motor.intake_intake()),
            follow(follower, path3()),
            waitMs(3000),
            instant(() -> intake_motor.intake_stop())
        );
    }
    public Command intakeflower2() {
        return sequential(
            instant(() -> intake_motor.intake_intake()),
            follow(follower, path6()),
            waitMs(3000),
            instant(() -> intake_motor.intake_stop())
        );
    }
    public Command autoRoutine() {
        return sequential(
                //shoot1
                follow(follower, path1()),
                shootfar(),

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
        return Paths.line(point7, point8).reverseTangent();
    }
}
