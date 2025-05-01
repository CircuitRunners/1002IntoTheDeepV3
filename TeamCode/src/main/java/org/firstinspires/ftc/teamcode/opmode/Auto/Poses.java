package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.localization.Pose;

public class Poses {
    // Preload Pathchain

    public static final double farX = 55;
    public static final double closeX = 30;
    public static final double wallY = 9;
    public static final double intakeX = 8+1;//idk about the +1
    public static final double subX = 42;
    public static final double subY = 77;

    public static final Pose startPose = new Pose(7.3285, 65.83, 0);

    public static final Pose preloadPose = new Pose(subX + 1, 65.83, 0);

    // Combined Push Pathchain

    // pre push
    public static final Pose spline1Control = new Pose(0, 65.83, 0);
    public static final Pose spline1End = new Pose(farX - 2, 32.5, 0);

    // push 1
    public static final Pose spline2Control = new Pose(farX, 20, 0);
    public static final Pose spline2End = new Pose(closeX, 20, 0);

    // return to push 2

    public static final Pose spline3Control = new Pose(farX+1, 28, 0);
    public static final Pose spline3End = new Pose(farX + 1, 16, 0);

    // push 2

    public static final Pose pushSample2End = new Pose(closeX, 16, 0);

    public static final Pose spline4Control = new Pose(farX, 16.5, 0);
    public static final Pose spline4End = new Pose(farX, wallY, 0);

    public static final Pose pushSample3End = new Pose(closeX + 5, wallY, 0);

    public static final Pose spline5Control = new Pose(closeX + 5, 36-2, 0);
    public static final Pose spline5End = new Pose(intakeX, 36-1.5, 0);

    public static final Pose scoreEnd = new Pose (subX, subY, 0);

}
