package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;


import java.util.List;

import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

@Autonomous(name = "Delayed Right", preselectTeleOp = "aaTeleop")
public class DelayRight extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;
    private int specCounter = 0;

    private double power;

    private static final double SCORE_VEL_THRESHOLD = 0.8;
    private static final double INTAKE_VEL_THRESHOLD = 0.5;

    private static final int SLIDE_SCORE = 450;
    private static final int SLIDE_SAFE = 350;

    private PathChain preload, push1, push2, push3, score, park, nudge3, intake, push4;


    private final double startingX = 7.3285;
    private final double startingY = 65.83;
    private final Pose preScorePose = new Pose(startingX + 40, startingY + 17, Math.toRadians(0));
    private final Pose scorePose = new Pose(startingX +56,startingY + 8, Math.toRadians(90));
    private final Pose intakePose = new Pose(startingX +.610, startingY + 18.3, Math.toRadians(0));
    private final Pose preloadControlPose = new Pose(startingX +12,startingY + 20, Math.toRadians(0));
    private final Pose preIntakePose = new Pose(startingX + 11, startingY + 17, Math.toRadians(0));
    private final Pose push1ControlPose = new Pose(44, 95.66, Math.toRadians(0));
    private final Pose push2ControlPose = new Pose(42, 98.66, Math.toRadians(0));
    private final Pose push3ControlPose = new Pose(startingX + 24, startingY + 32, Math.toRadians(0));
    private final Pose dropPose = new Pose(startingX + 9, startingY + 10, Math.toRadians(0));
    private final Pose turnPose = new Pose(startingX + 55, startingY + 16, Math.toRadians(0));
    private final Pose parkPose = new Pose(startingX + 10, startingY + 17, Math.toRadians(0));
    private final Pose nudge3Pose = new Pose(startingX + 58, startingY + 18, Math.toRadians(100));
    private final Pose pre1Pose = new Pose(startingX + 44.7, startingY + 21, Math.toRadians(0));
    private final Pose pre3Pose = new Pose(startingX + 55, startingY + 18.5, Math.toRadians(90));
    private final Pose drop3Pose = new Pose(startingX + 7, startingY + 10, Math.toRadians(0));
    private final Pose turn1Pose = new Pose(startingX +57,startingY + 9, Math.toRadians(40));
    private final Pose pre2Pose = new Pose(startingX + 56, startingY + 21, Math.toRadians(0));
    private final Pose mainPreScorePose = new Pose(startingX + 40, startingY + 15, Math.toRadians(0));
    private final Pose mainTurnPose = new Pose(startingX + 55, startingY + 15, Math.toRadians(0));
    private final Pose preNudge = new Pose(startingX + 55.3, startingY + 18.2, Math.toRadians(100));

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(Poses.startPose), pointFromPose(preloadControlPose), pointFromPose(preScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pointFromPose(preScorePose), pointFromPose(turnPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(new BezierLine(pointFromPose(turnPose), pointFromPose(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        push1 = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(scorePose), pointFromPose(turn1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .addPath(new BezierLine(pointFromPose(turn1Pose), pointFromPose(pre1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(new BezierCurve(pointFromPose(pre1Pose), pointFromPose(push1ControlPose), pointFromPose(dropPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))
                .addPath(new BezierLine(pointFromPose(dropPose), pointFromPose(preIntakePose)))
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
                .build();

        push2 = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(pre2Pose), pointFromPose(push2ControlPose), pointFromPose(dropPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(20))
                .addPath(new BezierLine(pointFromPose(dropPose), pointFromPose(preIntakePose)))
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
                .build();

        push3 = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(scorePose), pointFromPose(pre3Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-100))
                .addPath(new BezierLine(pointFromPose(pre3Pose), pointFromPose(pre2Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-100), Math.toRadians(-45))
                .addPath(new BezierCurve(pointFromPose(pre2Pose), pointFromPose(push2ControlPose), pointFromPose(dropPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))
                .addPath(new BezierLine(pointFromPose(dropPose), pointFromPose(preIntakePose)))
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(intakePose), pointFromPose(mainPreScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pointFromPose(mainPreScorePose), pointFromPose(mainTurnPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .addPath(new BezierLine(pointFromPose(turnPose), pointFromPose(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(scorePose), pointFromPose(turnPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .addPath(new BezierLine(pointFromPose(turnPose), pointFromPose(parkPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        nudge3 = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(scorePose), pointFromPose(preNudge)))
                .setLinearHeadingInterpolation(Math.toRadians(-100), Math.toRadians(-92))
                .addPath(new BezierLine(pointFromPose(preNudge), pointFromPose(nudge3Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(-92), Math.toRadians(-48))
                .build();

        intake = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(preIntakePose), pointFromPose(intakePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        push4 = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(scorePose), pointFromPose(turnPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .addPath(new BezierLine(pointFromPose(turnPose), pointFromPose(preIntakePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    //gee i sure hope this works first try
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 8) {
                        setPathState();
                    }
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(preload, true);
                    setPathState();
                }
                break;
            case 2:
                preScore();
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    if (specCounter == 1) {
                        follower.followPath(push1, true);
                        setPathState();
                    }
                    else if (specCounter == 2) {
                        follower.followPath(push2, true);
                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                            follower.setMaxPower(0.8);
                        }
                        setPathState();
                    }
                    else if (specCounter == 3) {
                        follower.followPath(push3, true);
                        setPathState();
                    }
                    else if (specCounter == 4) {
                        follower.setMaxPower(1);
                        follower.followPath(push4, true);
                        setPathState();
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(intake);
                    setPathState();
                }
                break;
            case 5:
                intakePrep();
                break;
            case 6:
                intake();
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(score, true);
                    setPathState();
                }
                break;
            case 8:
                score();
                break;
            case 9:
                if (!follower.isBusy()) {
                    if (specCounter == 5) {
                        follower.setMaxPower(1);
                        follower.followPath(park, true);
                        setPathState(-1);
                    }
                    else if (specCounter == 2) {
                        setPathState();
                    }
                    else {
                        setPathState(2);
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(nudge3, true);
                    setPathState(2);
                }

            default:
                if (follower.getPose().getY() < 65 && follower.getPose().getX() < 50 && specCounter == 5) {
                    requestOpModeStop();
                }
                intakePrep();
        }
    }

    public void score() {
        if (follower.getPose().getX() > 10 && follower.getPose().getX() < 20 && follower.isBusy()) {
            endEffector.setSpecScore();
            deposit.setSlideTarget(SLIDE_SCORE);
        }

        if (follower.getVelocityMagnitude() < SCORE_VEL_THRESHOLD && follower.getPose().getX() > 35) {
            endEffector.openClaw();
            deposit.setSlideTarget(SLIDE_SAFE);
            specCounter++;
            follower.breakFollowing();
            setPathState();
        }
    }
    public void preScore() {
        if (follower.getPose().getX() > 5 && follower.getPose().getX() < 20 && follower.isBusy()) {
            endEffector.setSpecScore();
            deposit.setSlideTarget(SLIDE_SCORE);
        }

        if (follower.getVelocityMagnitude() < SCORE_VEL_THRESHOLD && follower.getPose().getX() > 55) {
            endEffector.openClaw();
            deposit.setSlideTarget(SLIDE_SAFE);
            specCounter++;
            follower.breakFollowing();
            setPathState();
        }
    }

    public void intakePrep() {
        if (pathTimer.getElapsedTimeSeconds() > 0.25) {
            deposit.setSlideTarget(0);
            if (deposit.slidesReached) {
                if (specCounter >= 5) {
                    endEffector.setSpecScore();
                } else {
                    endEffector.setWallIntakePositionAlt();
                }
            }
            if (specCounter >= 5) {
                setPathState(-1);
            } else {
                setPathState();
            }
        }
    }

    public void intake() {
        if (follower.getVelocityMagnitude() < INTAKE_VEL_THRESHOLD && follower.getPose().getX() < 10 && endEffector.clawPosition != 0.13) {
            endEffector.closeClaw();
            pathTimer.resetTimer();
        }

        if (endEffector.clawPosition == 0.13 && pathTimer.getElapsedTime() > 200) {
            endEffector.setSpecScore();
            setPathState();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Specs Scored", specCounter);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Power", power);


        telemetry.update();

//        if (specCounter == 5) {
//            follower.setMaxPower(1);
//            power = 1;
//        }
//        else if (specCounter == 0) {
//            follower.setMaxPower(0.8);
//            power = 0.8;
//        }
//        else if (follower.getPose().getX() < 12 && follower.getPose().getY() < startingY - 16) {
//                follower.setMaxPower(0.35);
//                power = 0.35;
//            }
////        else if (follower.getPose().getX() < 24 && follower.getPose().getY() < 48 && follower.getPose().getY() > 24 && follower.getPose().getY() > 10 && follower.getVelocity().getXComponent() < 0) {
////            follower.setMaxPower(0.35);
////            power = 0.35;
////            }
//        else if (specCounter == 1) {
//            follower.setMaxPower(0.8);
//            power = 0.8;
//        }
//        else {
//            follower.setMaxPower(0.8);
//            power = 0.8;
//        }

//        follower.setMaxPower(0.5);

        deposit.update();

        autonomousPathUpdate();
    }

    @Override
    public void stop() {
        endEffector.openClaw();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();

        // Initialize follower, slides, etc. as usual
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        // If needed, set a starting pose (only if your system requires it)
        // follower.setStartingPose(new Pose(0,0, 0));
        follower.setStartingPose(Poses.startPose);

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);
        endEffector.setLight(0.5);
        endEffector.override = false;

        endEffector.setAutoIdle();
        deposit.setPivotTarget(121);
        deposit.setSlideTarget(0);

        follower.setMaxPower(0.8);

        // Build our newly incorporated multi-step path:
        buildPaths();

//        if (!deposit.slideLimit.isPressed()) {
//            throw new IllegalArgumentException("Zero slides before init");
//        }
    }

    @Override
    public void init_loop() {
        endEffector.setAutoIdle();
        endEffector.closeClaw();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        endEffector.closeClaw();
        setPathState(0);
    }

    private Point pointFromPose(Pose pose) {
        // Replace getX() and getY() with your actual accessor methods if needed.
        return new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
    }


}


