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

@Autonomous(name = "CRI Left", preselectTeleOp = "aaTeleop")
public class CRISpecLeft extends OpMode {
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

    private PathChain preload, push1, push2, push3, score;

    private final double startingX = 7.3285;
    private final double startingY = 65.83;
    private final Pose preScorePose = new Pose(startingX + 40.5, startingY - 17, Math.toRadians(0));
    private final Pose scorePose = new Pose(startingX +58,startingY -11,Math.toRadians(270));
    private final Pose intakePose = new Pose(startingX +.610, startingY -18.375, Math.toRadians(0));
    private final Pose preloadControlPose = new Pose(startingX +12,startingY -20, Math.toRadians(0));
    private final Pose scoreControlPose = new Pose(startingX +60, startingY -18, Math.toRadians(0));

    private final Pose push1ControlPose = new Pose(56, 29, Math.toRadians(0));
    private final Pose push2ControlPose = new Pose(63, 29, Math.toRadians(0));
    private final Pose push3ControlPose = new Pose(70, 29, Math.toRadians(0));



    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(Poses.startPose), pointFromPose(preloadControlPose), pointFromPose(preScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(pointFromPose(preScorePose), pointFromPose(scoreControlPose), pointFromPose(scorePose)))
                .setTangentHeadingInterpolation()
                .build();

        push1 = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(scorePose), pointFromPose(push1ControlPose), pointFromPose(intakePose)))
                .setTangentHeadingInterpolation()
                .build();

        push2 = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(scorePose), pointFromPose(push2ControlPose), pointFromPose(intakePose)))
                .setTangentHeadingInterpolation()
                .build();

        push3 = follower.pathBuilder()
                .addPath(new BezierCurve(pointFromPose(scorePose), pointFromPose(push3ControlPose), pointFromPose(intakePose)))
                .setTangentHeadingInterpolation()
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierLine(pointFromPose(intakePose), pointFromPose(preScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(pointFromPose(preScorePose), pointFromPose(scoreControlPose), pointFromPose(scorePose)))
                .setTangentHeadingInterpolation()
                .build();
    }

    //gee i sure hope this works first try
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(preload, true);
                    setPathState();
                }
                break;
            case 1:
                preScore();
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (specCounter == 1) {
                        follower.followPath(push1, true);
                        setPathState();
                    }
                    else if (specCounter == 2) {
                        follower.followPath(push2, true);
                        setPathState();
                    }
                    else if (specCounter >= 3) {
                        follower.followPath(push3, true);
                        setPathState();
                    }
                }
                break;
            case 3:
                intakePrep();
                break;
            case 4:
                intake();
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(score, true);
                    setPathState();
                }
                break;
            case 6:
                score();
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(push3, true);
                    if (specCounter == 5) {
                        setPathState(-1);
                    }
                    else {
                        setPathState(2);
                    }
                }
                break;

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

        if (follower.getVelocityMagnitude() < SCORE_VEL_THRESHOLD && follower.getPose().getX() > 35) {
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

        if (specCounter == 5) {
            follower.setMaxPower(1);
            power = 1;
        } else if (follower.getPose().getX() < 40 && follower.getPose().getY() < 60 && follower.getPose().getY() > 28 && follower.getVelocity().getXComponent() < 0 && specCounter != 1) {
            follower.setMaxPower(0.35);
            power = 0.35;
        } else if (follower.getPose().getX() < 24 && follower.getPose().getY() < 48 && follower.getPose().getY() > 24 && follower.getPose().getY() > 10 && follower.getVelocity().getXComponent() < 0) {
            follower.setMaxPower(0.35);
            power = 0.35;
        } else if (specCounter == 1) {
            follower.setMaxPower(1);
            power = 1;
        }else {
            follower.setMaxPower(1);
            power = 1;
        }

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

