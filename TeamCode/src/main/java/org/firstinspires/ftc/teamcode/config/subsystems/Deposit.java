package org.firstinspires.ftc.teamcode.config.subsystems;

import android.text.method.Touch;

import com.seattlesolvers.solverslib.controller.PIDFController;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;



public class Deposit {

    public DcMotorEx rightLift, leftLift, pivot;
    public AnalogInput pivotEncoder;
    public TouchSensor slideLimit;

    private static final double[] autoPivotCoefficients = {0.035,0,0.001, 0.0025};
    private static final double[] teleopPivotCoefficients = {0.032,0,0.001, 0.0025};


    private static final double[] autoSlideCoefficients = {0.06,0,0.0016, 0};
    private static final double[] teleopSlideCoefficients = {0.04,0,0.0016, 0};

    // private static final double[] teleopSlideCoefficients = {0.08,0,0.0024, 0}; - hannah bucket changes

    //    private static final double[] teleopSlideCoefficients = {0.0125,0,0.0002, 0.0025};


    public PIDFController slidePIDF;
    public PIDFController pivotPIDF;
    private static double slideF;
    private static double pivotF;
    public double slideTarget = 0;
    public double pivotTarget = 0;
    public boolean slidesReached;
    public boolean pivotReached;
    // Between retracted and extended
    public boolean slidesRetracted;
    public double pivotPos;
    public double liftPos;
    public boolean pidfActive = true;

    public Deposit(HardwareMap hardwareMap, Telemetry telemetry, Boolean auto) {
        if (auto) {
            slidePIDF = new PIDFController(autoSlideCoefficients[0], autoSlideCoefficients[1], autoSlideCoefficients[2], autoSlideCoefficients[3]);
            pivotPIDF = new PIDFController(autoPivotCoefficients[0], autoPivotCoefficients[1], autoPivotCoefficients[2], autoPivotCoefficients[3]);
            slideF = autoSlideCoefficients[3];
            pivotF = autoPivotCoefficients[3];
        } else {
            slidePIDF = new PIDFController(teleopSlideCoefficients[0], teleopSlideCoefficients[1], teleopSlideCoefficients[2], teleopSlideCoefficients[3]);
            pivotPIDF = new PIDFController(teleopPivotCoefficients[0], teleopPivotCoefficients[1], teleopPivotCoefficients[2], teleopPivotCoefficients[3]);
            slideF = teleopSlideCoefficients[3];
            pivotF = teleopPivotCoefficients[3];
        }

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        slideLimit = hardwareMap.get(TouchSensor.class, "slide_limit");

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slidePIDF.setTolerance(15);
        pivotPIDF.setTolerance(1);
        setSlideTarget(Math.round((float) rightLift.getCurrentPosition() / 42) * -1);
        setPivotTarget(pivotPos());
    }

    public void setSlideTarget(double target) {
        this.slideTarget = Range.clip(target, 0, 1250);
        slidePIDF.setSetPoint(slideTarget);
    }

    public void setPivotTarget(double target) {
        this.pivotTarget = Range.clip(target, 0, 121);
        pivotPIDF.setSetPoint(pivotTarget);
    }



    public void update() {
        liftPos = liftPos();
        pivotPos = pivotPos();
        /// slidePIDF.setF(slideF * Math.sin(Math.toRadians(pivotPos)));
        double liftPower = slidePIDF.calculate(liftPos, slideTarget);
        slidesReached = slidePIDF.atSetPoint() || (liftPos >= slideTarget && slideTarget == 1050);
        slidesRetracted = slideTarget <= 0 && slideLimit.isPressed();

        pivotPIDF.setF(pivotF * Math.cos(Math.toRadians(pivotPos)) * ((double) liftPos / 1250));
        double pivotPower = pivotPIDF.calculate(pivotPos, pivotTarget);
        pivotReached = pivotPIDF.atSetPoint();

        // Just make sure it gets to fully retracted if target is 0
        if (slideTarget == 0 && !slidesReached) {
            liftPower -= 0.1;
        } else if (slideTarget >= 1050 && !slidesReached) {
            liftPower += 0.6;
        }

        if (pidfActive) {
            if (slidesRetracted) {
                rightLift.setPower(0);
                leftLift.setPower(0);
            } else if (pivotPos <= 10 && slidesReached) {
                rightLift.setPower(0);
                leftLift.setPower(0);
            } else {
                rightLift.setPower(liftPower);
                leftLift.setPower(liftPower);
            }
        } else {
            if (slideLimit.isPressed()) {
                rightLift.setPower(0);
                leftLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidePIDF.reset();
                pidfActive = true;
            } else {
                rightLift.setPower(-1);
                leftLift.setPower(-1);
            }
        }


        if ((slideTarget < 500 && pivotTarget == 121 && pivotReached) || (pivotTarget <= 12 && pivotReached)) {
            pivotPower = 0;
        }

        pivot.setPower(pivotPower);
    }

    public int liftPos() {
        return Math.round((float) rightLift.getCurrentPosition() / 42) * -1;
    }

    public int pivotPos() {
        // int pos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360)) % 360 - 168;
        int pos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360) + (15) % 360);

        if (pos >= 360) {
            pos -= 360;
        } else if (pos < 0) {
            pos += 360;
        }
        if (pos > 345) {
            pos = 0;
        }
        return pos;
    }
}