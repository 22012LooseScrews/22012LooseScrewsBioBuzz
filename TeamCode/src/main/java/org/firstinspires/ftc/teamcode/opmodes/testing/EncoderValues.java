package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;
import java.util.function.Supplier;

/**
 * Driver Hub version of pedro/procedures/ThreeWheelTuner: same measurements and same math, but you
 * step through it with gamepad1 instead of the Pedro tuning UI.
 *
 * A = start / save a step, B = re-zero the current step.
 *   1. Push FORWARD exactly PUSH_DISTANCE in -> left + right ticks per inch and directions
 *   2. Push LEFT exactly PUSH_DISTANCE in    -> strafe ticks per inch and direction
 *   3. Rotate 180 deg CCW about robot center -> left pod Y offset
 *   4. Rotate 180 deg CCW about robot center -> right pod Y offset
 *   5. Rotate 360 deg CCW                    -> turn multiplier (and strafe pod X)
 * Then it shows the finished ThreeWheelConfig to copy into pedro/Constants.java.
 */
@TeleOp(name = "EncoderValues", group = "Pedro Tuning")
public class EncoderValues extends LinearOpMode {

    // Must match the robot configuration. Left/right = parallel (forward) pods, strafe = perpendicular pod.
    private static final String LEFT_ENCODER = "lf";
    private static final String RIGHT_ENCODER = "rr";
    private static final String STRAFE_ENCODER = "lr";
    private static final String[] DRIVE_MOTORS = {"lf", "lr", "rf", "rr"};

    private static final double PUSH_DISTANCE = 48.0; // inches (2 tiles); push exactly this far in steps 1 and 2

    private static final String TAG = "EncoderValues";

    private DcMotorEx leftEncoder, rightEncoder, strafeEncoder;

    // Step 1-2 working values
    private int leftStart, rightStart, strafeStart;
    private int leftTicks, rightTicks, strafeTicks;

    // Results carried from step to step
    private double leftDirection, rightDirection, strafeDirection;
    private double forwardTicksToInches, strafeTicksToInches;
    private double leftPodY, rightPodY, leftStrafeTravel, rightStrafeTravel;

    // Step 3-5 working values
    private ThreeWheelLocalizer localizer;
    private Pose pose;
    private double startHeading, turnedRadians;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Let the wheels spin freely so the robot is easy to push by hand
        for (String name : DRIVE_MOTORS) {
            DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
            if (motor == null) continue;
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        leftEncoder = encoder(LEFT_ENCODER);
        rightEncoder = encoder(RIGHT_ENCODER);
        strafeEncoder = encoder(STRAFE_ENCODER);

        while (opModeInInit()) {
            telemetry.addLine("Three-wheel odometry tuner. Press START, then follow the steps.");
            telemetry.addLine("A = start / save a step,  B = re-zero the current step");
            telemetry.addLine();
            telemetry.addData("Left encoder (" + LEFT_ENCODER + ")", leftEncoder.getCurrentPosition());
            telemetry.addData("Right encoder (" + RIGHT_ENCODER + ")", rightEncoder.getCurrentPosition());
            telemetry.addData("Strafe encoder (" + STRAFE_ENCODER + ")", strafeEncoder.getCurrentPosition());
            telemetry.update();
        }
        if (isStopRequested()) return;
        gamepad1.aWasPressed(); // throw away any presses from init
        gamepad1.bWasPressed();

        // ---------- Step 1: forward push -> left + right resolution and direction ----------
        if (!runStep("Step 1/5: Forward push",
                "Push the robot FORWARD exactly " + PUSH_DISTANCE + " in. Keep it straight, don't turn.",
                () -> {
                    leftStart = leftEncoder.getCurrentPosition();
                    rightStart = rightEncoder.getCurrentPosition();
                },
                () -> String.format(Locale.US, "Left ticks: %d   Right ticks: %d",
                        leftEncoder.getCurrentPosition() - leftStart,
                        rightEncoder.getCurrentPosition() - rightStart),
                () -> {
                    leftTicks = leftEncoder.getCurrentPosition() - leftStart;
                    rightTicks = rightEncoder.getCurrentPosition() - rightStart;
                    if (leftTicks == 0) return "Left encoder (" + LEFT_ENCODER + ") didn't change.";
                    if (rightTicks == 0) return "Right encoder (" + RIGHT_ENCODER + ") didn't change.";
                    return null;
                })) return;

        double leftTicksPerInch = Math.abs(leftTicks / PUSH_DISTANCE);
        double rightTicksPerInch = Math.abs(rightTicks / PUSH_DISTANCE);
        leftDirection = leftTicks < 0 ? Encoder.REVERSE : Encoder.FORWARD;
        rightDirection = rightTicks < 0 ? Encoder.REVERSE : Encoder.FORWARD;

        // ---------- Step 2: left push -> strafe resolution and direction ----------
        if (!runStep("Step 2/5: Strafe push",
                "Push the robot LEFT exactly " + PUSH_DISTANCE + " in. Keep it straight, don't turn.",
                () -> strafeStart = strafeEncoder.getCurrentPosition(),
                () -> String.format(Locale.US, "Strafe ticks: %d",
                        strafeEncoder.getCurrentPosition() - strafeStart),
                () -> {
                    strafeTicks = strafeEncoder.getCurrentPosition() - strafeStart;
                    return strafeTicks == 0 ? "Strafe encoder (" + STRAFE_ENCODER + ") didn't change." : null;
                })) return;

        double strafeTicksPerInch = Math.abs(strafeTicks / PUSH_DISTANCE);
        strafeDirection = strafeTicks < 0 ? Encoder.REVERSE : Encoder.FORWARD;

        // Forward resolution is the harmonic mean of the two parallel pods (same as ThreeWheelTuner)
        double forwardTicksPerInch = 2.0 / (1.0 / leftTicksPerInch + 1.0 / rightTicksPerInch);
        forwardTicksToInches = 1.0 / forwardTicksPerInch;
        strafeTicksToInches = 1.0 / strafeTicksPerInch;

        // ---------- Step 3: rotate 180 -> left pod offset ----------
        if (!runStep("Step 3/5: Left pod offset",
                "Rotate exactly 180 deg COUNTERCLOCKWISE about the robot center. Keep the center in place.",
                () -> startLocalizer(config(0.0, -1.0, 0.0)),
                this::poseReadout,
                () -> {
                    pose = localizer.pose();
                    if (pose.x() == 0.0) return "Left pod (" + LEFT_ENCODER + ") didn't travel.";
                    leftPodY = -pose.x() / Math.PI;
                    leftStrafeTravel = pose.y() / Math.PI;
                    return null;
                })) return;

        // ---------- Step 4: rotate 180 -> right pod offset ----------
        if (!runStep("Step 4/5: Right pod offset",
                "Rotate exactly 180 deg COUNTERCLOCKWISE about the robot center. Keep the center in place.",
                () -> startLocalizer(config(1.0, 0.0, 0.0)),
                this::poseReadout,
                () -> {
                    pose = localizer.pose();
                    if (pose.x() == 0.0) return "Right pod (" + RIGHT_ENCODER + ") didn't travel.";
                    rightPodY = -pose.x() / Math.PI;
                    rightStrafeTravel = pose.y() / Math.PI;
                    return null;
                })) return;

        // ---------- Step 5: rotate 360 -> turn multiplier ----------
        // Uses the measured pod offsets and forwardTicksToInches as the starting turn multiplier
        if (!runStep("Step 5/5: Turn multiplier",
                "Rotate exactly 360 deg COUNTERCLOCKWISE. Go by the robot, not the number on screen.",
                () -> {
                    startLocalizer(config(leftPodY, rightPodY, forwardTicksToInches));
                    startHeading = localizer.getTotalHeading();
                },
                () -> {
                    localizer.update();
                    return String.format(Locale.US, "Heading (uncorrected): %.1f deg",
                            Math.toDegrees(localizer.getTotalHeading() - startHeading));
                },
                () -> {
                    turnedRadians = localizer.getTotalHeading() - startHeading;
                    return turnedRadians <= 0.0 ? "No counterclockwise rotation measured." : null;
                })) return;

        double turnTicksToRadians = forwardTicksToInches * (2.0 * Math.PI) / turnedRadians;
        double strafePodX = (leftStrafeTravel + rightStrafeTravel) / 2.0 * turnTicksToRadians / strafeTicksToInches;

        // ---------- Results ----------
        String code = "public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {\n" +
                "    c.leftEncoderName.set(\"" + LEFT_ENCODER + "\");\n" +
                "    c.rightEncoderName.set(\"" + RIGHT_ENCODER + "\");\n" +
                "    c.strafeEncoderName.set(\"" + STRAFE_ENCODER + "\");\n" +
                "    c.leftPodY.set(" + leftPodY + ");\n" +
                "    c.rightPodY.set(" + rightPodY + ");\n" +
                "    c.strafePodX.set(" + strafePodX + ");\n" +
                "    c.forwardTicksToInches.set(" + forwardTicksToInches + ");\n" +
                "    c.strafeTicksToInches.set(" + strafeTicksToInches + ");\n" +
                "    c.turnTicksToRadians.set(" + turnTicksToRadians + ");\n" +
                "    c.leftEncoderDirection.set(" + direction(leftDirection) + ");\n" +
                "    c.rightEncoderDirection.set(" + direction(rightDirection) + ");\n" +
                "    c.strafeEncoderDirection.set(" + direction(strafeDirection) + ");\n" +
                "});";
        RobotLog.ii(TAG, "%s", code); // backup copy in the robot log (logcat tag "EncoderValues")

        while (opModeIsActive()) {
            telemetry.addLine("DONE. Copy this into pedro/Constants.java:");
            telemetry.addLine(code);
            telemetry.addLine();
            telemetry.addData("leftTicksPerInch", leftTicksPerInch);
            telemetry.addData("rightTicksPerInch", rightTicksPerInch);
            telemetry.addData("forwardTicksPerInch", forwardTicksPerInch);
            telemetry.addData("strafeTicksPerInch", strafeTicksPerInch);
            telemetry.update();
        }
    }

    /**
     * Runs one step: wait for A (robot lined up), zero, show the live reading, A saves, B re-zeros.
     * If save returns an error message the step starts over. Returns false only if Stop was pressed.
     */
    private boolean runStep(String title, String instructions,
                            Runnable zero, Supplier<String> live, Supplier<String> save) {
        String error = null;
        while (opModeIsActive()) {
            telemetry.addLine(title);
            telemetry.addLine(instructions);
            if (error != null) telemetry.addLine("Didn't save: " + error + " Try again.");
            telemetry.addLine("Line the robot up, then press A to start.");
            telemetry.update();
            if (!gamepad1.aWasPressed()) continue;

            zero.run();
            while (opModeIsActive()) {
                String reading = live.get();
                if (gamepad1.bWasPressed()) {
                    zero.run();
                    continue;
                }
                if (gamepad1.aWasPressed()) break;
                telemetry.addLine(title);
                telemetry.addLine(instructions);
                telemetry.addLine(reading);
                telemetry.addLine("A = save and continue,  B = re-zero");
                telemetry.update();
            }
            if (!opModeIsActive()) return false;

            error = save.get();
            if (error == null) return true;
        }
        return false;
    }

    private void startLocalizer(ThreeWheelConfig config) {
        localizer = new ThreeWheelLocalizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();
    }

    private String poseReadout() {
        localizer.update();
        Pose p = localizer.pose();
        return String.format(Locale.US, "Parallel pod travel: %.2f in   Strafe pod travel: %.2f in", p.x(), p.y());
    }

    private ThreeWheelConfig config(double leftY, double rightY, double turnTicksToRadians) {
        return new ThreeWheelConfig(c -> {
            c.leftEncoderName.set(LEFT_ENCODER);
            c.rightEncoderName.set(RIGHT_ENCODER);
            c.strafeEncoderName.set(STRAFE_ENCODER);
            c.leftPodY.set(leftY);
            c.rightPodY.set(rightY);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(forwardTicksToInches);
            c.strafeTicksToInches.set(strafeTicksToInches);
            c.turnTicksToRadians.set(turnTicksToRadians);
            c.leftEncoderDirection.set(leftDirection);
            c.rightEncoderDirection.set(rightDirection);
            c.strafeEncoderDirection.set(strafeDirection);
        });
    }

    /** Raw encoder counts, which is what Pedro's Encoder class works from. */
    private DcMotorEx encoder(String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        return motor;
    }

    private static String direction(double direction) {
        return direction == Encoder.REVERSE ? "Encoder.REVERSE" : "Encoder.FORWARD";
    }
}