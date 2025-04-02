package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp
public class pivotTuner extends OpMode {
    public static double liftP = 0.0125, liftI = 0.0, liftD = 0.0002, liftF = 0.0025;
    public static double pivotP = 0.03, pivotI = 0, pivotD = 0.001, pivotF = 0.002;

    public static int liftSetPoint = 0;
    public static int pivotSetPoint = 0;
    public static double maxPowerConstant = 1.03;
    private static final PIDFController slidePIDF = new PIDFController(liftP,liftI,liftD, liftF);
    private static final PIDFController pivotPIDF = new PIDFController(pivotP,pivotI,pivotD, pivotF);
    public ElapsedTime timer = new ElapsedTime();
    int liftPos = 0;
    int pivotPos = 0;
    DcMotorEx rightLift, leftLift, pivot;
    AnalogInput pivotEncoder;

    @Override
    public void init() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivot.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidePIDF.setTolerance(12);
        pivotPIDF.setTolerance(2);

        telemetry.addData("lift position", liftPos);
        telemetry.addData("lift set point", liftSetPoint);
        telemetry.addData("pivot position", pivotPos);
        telemetry.addData("pivot set point", pivotSetPoint);
        telemetry.addData("lift max power", (liftF * liftPos) + maxPowerConstant);
        telemetry.addData("pivot max power", (pivotF * pivotPos) + maxPowerConstant);

        telemetry.update();
    }

    @Override
    public void loop() {
        liftPos = Math.round((float) rightLift.getCurrentPosition() / 42) * -1;
<<<<<<< HEAD
        pivotPos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360) +  3) % 360;
=======
        pivotPos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360) + 75) % 360;
>>>>>>> 43

        slidePIDF.setPIDF(liftP, liftI, liftD, liftF * Math.sin(Math.toRadians(pivotPos)));
        pivotPIDF.setPIDF(pivotP, pivotI, pivotD, pivotF * Math.cos(Math.toRadians(pivotPos) * ((double) liftPos / 1100)));

        slidePIDF.setSetPoint(liftSetPoint);
        pivotPIDF.setSetPoint(pivotSetPoint);

        double liftMaxPower = maxPowerConstant;
        double liftPower = Range.clip(slidePIDF.calculate(liftPos, liftSetPoint), -liftMaxPower, liftMaxPower);

        double pivotMaxPower = maxPowerConstant;
        double pivotPower = Range.clip(pivotPIDF.calculate(pivotPos, pivotSetPoint), -pivotMaxPower, pivotMaxPower);

        // liftPower = (liftPower / Math.abs(liftPower)) * Math.sqrt(Math.abs(liftPower));
        //rightLift.setPower(liftPower);
        //leftLift.setPower(liftPower);

        // pivotPower = (pivotPower / Math.abs(pivotPower)) * Math.sqrt(Math.abs(pivotPower));

        //pivot.setPower(pivotPower);

        telemetry.addData("lift position", liftPos);
        telemetry.addData("lift set point", liftSetPoint);
        telemetry.addData("pivot position", pivotPos);
        telemetry.addData("pivot set point", pivotSetPoint);
        telemetry.addData("lift power", liftPower);
        telemetry.addData("pivot power", pivotPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }

<<<<<<< HEAD
}
=======
}
>>>>>>> 43ac9c9e17580f21c9daafc3a94799b68d5660ed
