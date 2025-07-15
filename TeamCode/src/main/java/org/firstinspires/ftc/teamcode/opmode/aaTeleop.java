package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.config.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;

@TeleOp
public class aaTeleop extends OpMode {

    // Hardware references
    private MecanumDrive drive;
    private Deposit slides;
    private EndEffector endEffector;
    private GoBildaPinpointDriver pinpoint;

    // Gamepad snapshots
    private GamepadEx player1;
    private GamepadEx player2;
    private AnalogInput distanceSensor;

    TriggerReader rightTriggerReader;

    TriggerReader leftTriggerReader;

    // State machine variables
    private int intakeState = -1;
    private int depositState = -1;
    private boolean specScoring = true;
    private boolean lowChamber = false;
    private int phase = 0;  // used in multi-phase intake logic
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime depositTimer = new ElapsedTime();

    private final static double PIVOT_DOWN = 6;
    private static boolean distance = false;

    @Override
    public void init() {
        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);
        distanceSensor = hardwareMap.get(AnalogInput.class, "distance");
        rightTriggerReader  = new TriggerReader(
                player1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        leftTriggerReader = new TriggerReader(
                player1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems
        drive = new MecanumDrive();
        slides = new Deposit(hardwareMap, telemetry, false);
        endEffector = new EndEffector(hardwareMap);

        endEffector.setLight(0.5);
        endEffector.override = false;
        drive.init(hardwareMap);

        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        // Reset timers
        intakeTimer.reset();
        depositTimer.reset();

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        player1.readButtons();
        player2.readButtons();

        leftTriggerReader.readValue();
        rightTriggerReader.readValue();


        // Handle Spec / Samp Deposit
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            specScoring = !specScoring;
            gamepad1.rumbleBlips(1);
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            lowChamber = !lowChamber;
        }

        // Handle LED override logic
        if (endEffector.override) {
            endEffector.setLight(0);
        } else if (endEffector.pin0() && endEffector.pin1()) {
            endEffector.setLight(0.388);
        } else if (endEffector.pin0()) {
            endEffector.setLight(0.611);
        } else if (endEffector.pin1()) {
            endEffector.setLight(0.29);
        } else {
            endEffector.setLight(0.5);
        }


        // 2) Field-relative drive / IMU Controls
        double forward = player1.getLeftY();
        double strafe  =  player1.getLeftX();
        double rotate  =  player1.getRightX();
        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // IMU Reset / Calibrate
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE) || player2.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            pinpoint.resetPosAndIMU();
        }
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE) || player2.wasJustPressed(GamepadKeys.Button.TRIANGLE)) {
            pinpoint.recalibrateIMU();
        }

        // 3) Circle button => reset states
        boolean circleJustPressed = player1.wasJustPressed(GamepadKeys.Button.CIRCLE) || player2.wasJustPressed(GamepadKeys.Button.CIRCLE);
        if (circleJustPressed) {
            endEffector.setIdlePosition();
            intakeState = -1;
            depositState = -1;
            slides.pidfActive = false;
            slides.setPivotTarget(90);
            slides.setSlideTarget(0);
        }

        boolean crossJustPressed = player1.wasJustPressed(GamepadKeys.Button.CROSS) || player2.wasJustPressed(GamepadKeys.Button.CROSS);
        boolean crossJustReleased = player1.wasJustReleased(GamepadKeys.Button.CROSS) || player2.wasJustReleased(GamepadKeys.Button.CROSS);

        if (crossJustPressed) {
            intakeState = -1;
            depositState = -1;
            slides.setPivotTarget(121);
            slides.setSlideTarget(700);
        }

        if (crossJustReleased) {
            intakeState = -1;
            depositState = -1;
            slides.setPivotTarget(121);
            slides.setSlideTarget(50);
        }


        // 4) Rising edge detection for bumpers/triggers
        boolean leftBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumperJustPressed = player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);




        boolean leftTriggerJustPressed = leftTriggerReader.wasJustPressed();
        boolean rightTriggerJustPressed = rightTriggerReader.wasJustPressed();

        // Toggle override with share
        boolean shareJustPressed = player1.wasJustPressed(GamepadKeys.Button.SHARE);
        boolean psJustPressed = player1.wasJustPressed(GamepadKeys.Button.PS);
        if (psJustPressed) {
            endEffector.override = !endEffector.override;
        }
        if (shareJustPressed) {
            distance = !distance;
        }

        // 5) Spec Intake (left bumper + / left trigger -)
        if (leftBumperJustPressed) {
            if (specScoring) {
                intakeState = (intakeState + 1) % 5;
            } else {
                intakeState = (intakeState + 1) % 8;
            }
        }
        if (leftTriggerJustPressed) {
            if (specScoring) {
                intakeState = (intakeState - 1) % 5;
            } else {
                intakeState = (intakeState - 1) % 8;
            }
        }

        // 6) Spec Deposit (right bumper + / right trigger -)
        if (rightBumperJustPressed) {
            depositState = (depositState + 1) % 5;
        }
        if (rightTriggerJustPressed) {
            depositState = (depositState - 1) % 5;
        }

        // If both states are active, reset them
        if (intakeState != -1 && depositState != -1) {
            intakeState = -1;
            depositState = -1;
        }

        // 7) DPAD Controls for Claw & Pivot/Wrist
        boolean dpadLeftJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRightJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);

        boolean dpadUpJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_UP);
        boolean dpadDownJustPressed = player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadUpDown = player1.isDown(GamepadKeys.Button.DPAD_UP);
        boolean dpadDownDown = player1.isDown(GamepadKeys.Button.DPAD_DOWN);

        if (dpadLeftJustPressed) {
            endEffector.openClaw();
        }
        if (dpadRightJustPressed) {
            endEffector.closeClaw();
        }

        // Pivot or Wrist control depending on pivot angle
        if (slides.pivotTarget > 45) {
            if (dpadUpDown) {
                endEffector.incrementPivotPosition(0.02);
            }
            if (dpadDownDown) {
                endEffector.decrementPivotPosition(0.02);
            }
        } else {
            if (dpadUpJustPressed) {
                endEffector.incrementWristPosition(0.25);
            }
            if (dpadDownJustPressed) {
                endEffector.decrementWristPosition(0.25);
            }
        }



        // More manual pivot/wrist control on gamepad2 (optional)
        if (player2.isDown(GamepadKeys.Button.DPAD_UP)) {
            endEffector.incrementWristPosition(0.02);
        }
        if (player2.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            endEffector.decrementWristPosition(0.02);
        }
        if (player2.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            endEffector.incrementPivotPosition(0.02);
        }
        if (player2.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            endEffector.decrementPivotPosition(0.02);
        }

        // 8) Intake State Machine
        switch (intakeState) {
            case 0:
                // pivot=10, slide=0, idle pos, open claw
                slides.setPivotTarget(PIVOT_DOWN);
                slides.setSlideTarget(0);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                intakeTimer.reset();
                phase = 0;
                break;

            case 1:
                // Lift to ~450
                slides.setSlideTarget(610);
                if (slides.liftPos > 380) {
                    endEffector.openClaw();
                    endEffector.setPreSubPickupPosition();
                } else if (slides.slideTarget > 300) {
                    endEffector.setIdlePosition();
                    endEffector.openClaw();
                }
                intakeTimer.reset();
                phase = 0;
                break;

            case 2:
                // Multi-phase logic
                switch (phase) {
                    case 0: // subPickup
                        if (intakeTimer.milliseconds() == 0) {
                            intakeTimer.reset();
                        }
                        endEffector.setSubPickupPosition();
                        if (intakeTimer.milliseconds() >= 175) {
                            endEffector.closeClaw();
                        }
                        if (intakeTimer.milliseconds() >= 200) {
                            phase = 1;
                            intakeTimer.reset();
                        }
                        break;
                    case 1: // deposit
                        endEffector.setSafeIdle();
                        if (!endEffector.either()) {
                            intakeState = 1;
                        } else {
                            slides.setPivotTarget(PIVOT_DOWN);
                            slides.setSlideTarget(0);
                        }
//
//                        slides.setPivotTarget(12);
//                        slides.setSlideTarget(0);
//                        if (intakeTimer.milliseconds() >= 200) {
//                            if (!endEffector.either()) {
//                                intakeState = 1;
//                            }
//                        }
                        break;
                }
                break;
            case 3:
                if (specScoring) {
                    slides.setPivotTarget(90);
                    endEffector.setObsDepositPosition();
                } else {
                    slides.setPivotTarget(95);
                    endEffector.setBucketScorePosition();
                }
                intakeTimer.reset();
                phase = 0;
                break;
            case 4:
                if (specScoring) {
                    endEffector.openClaw();
                } else {
                    intakeState++;
                }
                intakeTimer.reset();
                phase = 0;
                break;
            case 5:
                slides.setPivotTarget(95);
                //slides.setSlideTarget(515);
                slides.setSlideTarget(1215);
                intakeTimer.reset();
                phase = 0;
                break;
            case 6:
                endEffector.openClaw();
                if (intakeTimer.milliseconds() > 400) {
                    endEffector.setBucketSafeIdle();
                }

                if (intakeTimer.milliseconds() > 400) {
                    slides.setSlideTarget(0);
                }

                phase = 0;
                break;
            case 7:
                intakeTimer.reset();
                if (slides.liftPos() < 200) {
                    intakeState = (intakeState + 1) % 8;
                }
                phase = 0;
                break;

            default:
                intakeTimer.reset();
                phase = 0;
                break;
        }

        // 9) Deposit State Machine
        switch (depositState) {
            case 0:
                lowChamber = false;
                slides.setPivotTarget(121);
                slides.setSlideTarget(0);
                endEffector.setWallIntakePositionAlt();
                endEffector.openClaw();
                depositTimer.reset();
                break;

            case 1:
                if (!endEffector.pin0() && !endEffector.pin1()) {
                    depositState = 0;
                    depositTimer.reset();
                    break;
                }
                endEffector.closeClaw();
                if (depositTimer.milliseconds() > 300) {
                    depositState = 2;
                    depositTimer.reset();
                }                break;

            case 2:
                slides.setSlideTarget(0);
                if (lowChamber) {
                    endEffector.setLowChamberScore();
                }
                else {
                    endEffector.setSpecScore();
                }
                depositTimer.reset();
                break;
            case 3:
                if (lowChamber) {
                    depositState++;
                }
                else {
                    slides.setSlideTarget(480);
                }
                depositTimer.reset();
                break;
            case 4:
                endEffector.openClaw();
                if (lowChamber) {
                    break;
                }
                else if (endEffector.clawPosition > 0.5 && depositTimer.milliseconds() > 200) {
                    slides.setSlideTarget(350);
                }
                break;
            default:
                depositTimer.reset();
                break;
        }

        // Update deposit mechanics
        slides.update();

        // 10) Telemetry
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("End Effector Pivot Pos", endEffector.getPivotPosition());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Lift Pos", slides.liftPos);
        telemetry.addData("Lift Target", slides.slidePIDF.getSetPoint());
        telemetry.addData("Pivot Pos", slides.pivotPos);
        telemetry.addData("Pivot Target", slides.pivotTarget);
        telemetry.addData("specScoring", specScoring);
        telemetry.addData("Color Sensor pin0", endEffector.pin0());
        telemetry.addData("Color Sensor pin1", endEffector.pin1());

        telemetry.update();
    }

    /**
     * Configures the GoBilda Pinpoint device.
     */
    private void configurePinpoint() {
        pinpoint.setOffsets(6.03262717 * 25.4, 2.71126772 * 25.4);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    /**
     * Applies field-relative drive using the Pinpoint’s IMU heading.
     */
    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        if (distance && endEffector.clawPosition > 0.13 && depositState != -1 && distanceSensor.getVoltage() < 0.12) {
            forward /= 2;
            right /= 2;
        }
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

        // Convert driver input into field-centric motion
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // Convert to robot-centric forward & strafe
        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        // Drive with final inputs
        drive.drive(newForward, newRight, rotate);
        return pos;
    }
}
