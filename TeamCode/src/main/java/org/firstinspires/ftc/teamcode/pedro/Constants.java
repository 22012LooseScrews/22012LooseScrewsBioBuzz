package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lr");
                c.frontRightName.set("rf");
                c.backRightName.set("rr");
                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.manualBrakeMode.set(true);
            }
    );
//    public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(
//            c -> {
//                c.name.set("threewheel");
//                c.xPodOffset.set(2.187);
//                c.yPodOffset.set(-4.572);
//                c.
//                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//            }
//    );
    public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {
        c.leftEncoderName.set("lf");
        c.rightEncoderName.set("rr");
        c.strafeEncoderName.set("lr");
        c.leftPodY.set(7.3324880906225545);
        c.rightPodY.set(-6.342893654284258);
        c.strafePodX.set(-6.115732862126202);
        c.forwardTicksToInches.set(0.002129385350160356);
        c.strafeTicksToInches.set(0.001987464690747482);
        c.turnTicksToRadians.set(0.0021080705998447564);
        c.leftEncoderDirection.set(Encoder.FORWARD);
        c.rightEncoderDirection.set(Encoder.REVERSE);
        c.strafeEncoderDirection.set(Encoder.REVERSE);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.5168525231401863);
                Controller secondaryTranslationalForward = Controller.proportional(0.19096319674652792);
                Controller primaryTranslationalLateral = Controller.proportional(1.674417035358147);
                Controller secondaryTranslationalLateral = Controller.proportional(0.6186523533176392);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010087507203880004));
                c.brake.set(Controller.proportionalFeedforward(0.008574381123298003));

                c.headingFeedback.set(Controller.proportional(18.850337719462615));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07210790975283332, 0.008346921568181913));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11701231082182693, 0.20437049210383426));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015343770816734102, -0.001242290614521972));

                c.maxAchievableForwardVelocity.set(88.27248810428189);
                c.maxAchievableStrafeVelocity.set(63.19537627172035);
                c.naturalForwardDeceleration.set(54.88531732519427);
                c.naturalStrafeDeceleration.set(109.06179912325304);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new ThreeWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
