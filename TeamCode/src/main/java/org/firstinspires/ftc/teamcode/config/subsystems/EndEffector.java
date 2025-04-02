package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * The EndEffector class manages all of the servo components on the end effector.
 * It provides preset methods such as openClaw, closeClaw, setSubPickupPosition, etc.
 * Note that users of this class never directly control individual servo positions.
 */
@Config
public class EndEffector {

    // Configurable positions (visible on FTC Dashboard)
    public double armPosition = 0.5;    // must be within [0.2, 0.65]
    public double pivotPosition = 0.1;  // pivot allowed full range [0, 1]
    public double wristPosition = 0.45; // must be within [0.38, 0.7]
    public double clawPosition = 0.13;  // must be within [0.59, 0.74]
    public double lightPosition = 0.0;

    // Servo hardware references
    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo pivotServo;
    private final Servo wristServo;
    private final Servo clawServo;
    private final Servo light;

    // Digital channels (if used)
    private final DigitalChannel pin0;
    private final DigitalChannel pin1;

    // Override flag for sensor-based methods
    public boolean override = false;

    // Stores the most recent states of the digital channels
    public boolean pin0State;
    public boolean pin1State;

    /**
     * Constructs the EndEffector subsystem using the provided hardware map.
     *
     * @param hardwareMap The hardware map from the op mode.
     */
    public EndEffector(HardwareMap hardwareMap) {
        // Map servos
        armServoLeft  = hardwareMap.get(Servo.class, "armServoL");
        armServoRight = hardwareMap.get(Servo.class, "armServoR");
        pivotServo    = hardwareMap.get(Servo.class, "pivotServo");
        wristServo    = hardwareMap.get(Servo.class, "wristServo");
        clawServo     = hardwareMap.get(Servo.class, "clawServo");
        light         = hardwareMap.get(Servo.class, "light");

        // Configure servo directions (ensure left/right are oppositely oriented)
        armServoLeft.setDirection(Servo.Direction.FORWARD);
        armServoRight.setDirection(Servo.Direction.REVERSE);

        // Map digital channels
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
    }

    // --- Setters with Clipping ---

    /**
     * Sets both arm servos to a specified position.
     * The position is clipped to the range [0.2, 0.65].
     *
     * @param position The desired arm position.
     */
    public void setArmPosition(double position) {
        position = Range.clip(position, 0, 0.85);
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
        armPosition = position;
    }

    /**
     * Sets the pivot servo to a specified position.
     *
     * @param position The desired pivot position.
     */
    public void setPivotPosition(double position) {
        position = Range.clip(position, 0.24, 0.78);
        pivotServo.setPosition(position);
        pivotPosition = position;
    }

    /**
     * Sets the wrist servo to a specified position.
     * The position is clipped to the range [0.38, 0.7].
     *
     * @param position The desired wrist position.
     */
    public void setWristPosition(double position) {
        position = Range.clip(position, 0, 1);
        wristServo.setPosition(position);
        wristPosition = position;
    }

    /**
     * Sets the claw servo to a specified position.
     * The position is clipped to the range [0.59, 0.74].
     *
     * @param position The desired claw position.
     */
    public void setClawPosition(double position) {
        position = Range.clip(position, 0.13, 0.85);
        clawServo.setPosition(position);
        clawPosition = position;
    }

    /**
     * Sets the light servo to a specified position.
     *
     * @param position The desired light position.
     */
    public void setLight(double position) {
        if (lightPosition != position) {
            lightPosition = position;
            light.setPosition(position);
        }
    }

    // --- Getters for Servo Positions ---

    /**
     * Gets the current arm position (assumes both arm servos are identical).
     *
     * @return The arm servo position.
     */
    public double getArmPosition() {
        return armServoLeft.getPosition();
    }

    /**
     * Gets the current pivot position.
     *
     * @return The pivot servo position.
     */
    public double getPivotPosition() {
        return pivotServo.getPosition();
    }

    /**
     * Gets the current wrist position.
     *
     * @return The wrist servo position.
     */
    public double getWristPosition() {
        return wristServo.getPosition();
    }

    /**
     * Gets the current claw position.
     *
     * @return The claw servo position.
     */
    public double getClawPosition() {
        return clawServo.getPosition();
    }

    // --- Preset Position Methods ---

    /**
     * Initializes the end effector to a default starting state.
     */
    public void init() {
        setIdlePosition();
    }

    /**
     * Sets the end effector to an idle position.
     */
    public void setIdlePosition() {
        setPositions(0.54, 0.5, 0.5, clawPosition);
    }

    public void setAutoIdle() {
        setPositions(0.5, 0.5, 1, clawPosition);
    }

    /**
     * Sets the end effector to a safe idle position.
     */
    public void setSafeIdle() {
        setPositions(0.52, 0.5, 0.5, clawPosition);
    }

    public void setBucketSafeIdle() {
        setPositions(0.69, 0.65, 1, clawPosition);
    }

    /**
     * Sets the end effector to the bucket scoring position.
     */
//    public void setBucketScorePosition() {
//        setPositions(0.6, 0.45, 0.5, clawPosition);
//    }

    public void setBucketScorePosition() {
        setPositions(0.54, 0.45, 1, clawPosition);
    }

    /**
     * Sets the end effector to the pre-sub pickup position.
     */
    public void setPreSubPickupPosition() {
        // was 0.67 & 0.67
        setPositions(0.58, 0.75, wristPosition, clawPosition);
    }

    /**
     * Sets the end effector to the sub pickup position.
     */
    public void setSubPickupPosition() {
        setPositions(0.63, 0.75, wristPosition, clawPosition);
    }

    /**
     * Sets the end effector to the obstacle deposit position.
     */
    public void setObsDepositPosition() {
        setPositions(0.29, 0.43, 0, clawPosition);
    }

    /**
     * Sets the end effector to an alternate wall intake position.
     */
    public void setWallIntakePositionAlt() {
        setPositions(0.85, 0.55, 0, clawPosition);
    }

    /**
     * Sets the end effector to a special scoring position.
     */
    public void setSpecScore() {
        setPositions(0.37, 0.63, 1, 0.13);
    }

    /**
     * Opens the claw.
     */
    public void openClaw() {
        setClawPosition(0.85);
    }

    /**
     * Closes the claw.
     */
    public void closeClaw() {
        setClawPosition(0.13);
    }

    /**
     * Sets the positions of all servos at once.
     *
     * @param armPos   Position for the arm servos.
     * @param pivotPos Position for the pivot servo.
     * @param wristPos Position for the wrist servo.
     * @param clawPos  Position for the claw servo.
     */
    public void setPositions(double armPos, double pivotPos, double wristPos, double clawPos) {
        setArmPosition(armPos);
        setPivotPosition(pivotPos);
        setWristPosition(wristPos);
        setClawPosition(clawPos);
    }

    /**
     * Sets the positions of the arm, pivot, and wrist servos.
     *
     * @param armPos   Position for the arm servos.
     * @param pivotPos Position for the pivot servo.
     * @param wristPos Position for the wrist servo.
     */
    public void setPositions(double armPos, double pivotPos, double wristPos) {
        setArmPosition(armPos);
        setPivotPosition(pivotPos);
        setWristPosition(wristPos);
    }

    // --- Digital Channel Methods ---

    /**
     * Returns the state of digital pin0, accounting for the override.
     *
     * @return True if override is enabled or if pin0 is active.
     */
    public boolean pin0() {
        pin0State = pin0.getState();
        pin1State = pin1.getState();
        return override || pin0State;
    }

    /**
     * Returns the state of digital pin1, accounting for the override.
     *
     * @return True if override is enabled or if pin1 is active.
     */
    public boolean pin1() {
        pin0State = pin0.getState();
        pin1State = pin1.getState();
        return override || pin1State;
    }

    /**
     * Checks if either digital channel is active.
     *
     * @return True if override is enabled or if either pin is active.
     */
    public boolean either() {
        pin0State = pin0.getState();
        pin1State = pin1.getState();
        return override || pin0State || pin1State;
    }

    // --- Incremental Adjustment Methods (if needed) ---

    /**
     * Increments the wrist position by a specified step.
     * The new value is wrapped within the range [0.45, 1.0].
     *
     * @param step The amount to increment.
     */
    public void incrementWristPosition(double step) {
        setWristPosition(wrap(wristPosition + step, 0, 1.0));
    }

    /**
     * Decrements the wrist position by a specified step.
     * The new value is wrapped within the range [0.45, 1.0].
     *
     * @param step The amount to decrement.
     */
    public void decrementWristPosition(double step) {
        setWristPosition(wrap(wristPosition - step, 0, 1.0));
    }

    /**
     * Increments the pivot position by a specified step.
     *
     * @param step The amount to increment.
     */
    public void incrementPivotPosition(double step) {
        setPivotPosition(Range.clip(pivotPosition + step, 0.0, 1.0));
    }

    /**
     * Decrements the pivot position by a specified step.
     *
     * @param step The amount to decrement.
     */
    public void decrementPivotPosition(double step) {
        setPivotPosition(Range.clip(pivotPosition - step, 0.0, 1.0));
    }

    /**
     * Wraps a given value to remain within a specified range.
     *
     * @param value The value to wrap.
     * @param min   The minimum bound.
     * @param max   The maximum bound.
     * @return The wrapped value.
     */
    public static double wrap(double value, double min, double max) {
        double range = max - min;
        double wrappedValue = (value - min) % range;
        if (wrappedValue < 0) {
            wrappedValue += range;
        }
        return wrappedValue + min;
    }
}
