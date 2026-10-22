package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner;

public class Tuning {
    @Tuner
    public static Procedure ThreeWheelTuner() {
        return new ThreeWheelTuner();
    }
    @Tuner
    public static Procedure Tests() {
        return new Tests();
    }
}
