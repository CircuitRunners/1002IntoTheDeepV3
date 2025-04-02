package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.subsystems.MecanumDrive;

@TeleOp(name = "DRIVE ONLY")
public class GenericDriveOnly extends OpMode {

    MecanumDrive drive = new MecanumDrive();

    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;
    double rightStickYVal;


    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    public double leftSidePower;
    public double rightSidePower;


    //  SparkFunOTOS sparkfunOTOS;
    //   private Arm arm;
    // private EndEffector endEffector;

    private boolean shouldReverseInput = false; // Determines whether the input should be reversed
    private boolean joystickWasZero = true; // Tracks if the joystick was released
    //   private boolean prevTrianglePressed = false;

    //     private boolean prevDown = false;
    //   private int intakeState = 0;
    // private AnalogInput encoder;
    // AnalogInput

    public double speedMultiply = 1;

    private double powerThreshold = 0;

    private int driveMode = -1;


    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // encoder = hardwareMap.get(AnalogInput.class, "enc");
        drive.init(hardwareMap);

        driveMode = 1;

//        arm = new Arm(hardwareMap);
//        endEffector = new EndEffector(hardwareMap);
//
//        sparkfunOTOS = hardwareMap.get(SparkFunOTOS.class, HWValues.OTOS);
//        configureOTOS();
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {


        if (gamepad1.dpad_down == true) {
            speedMultiply = 0.50;
        } else if (gamepad1.dpad_left == true) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_right == true) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_up == true) {
            speedMultiply = 1;
        }

        drive();
        telemetry();
        //switchDriveMode();

//
//


        // Telemetry
    }


    public void drive() {

//        switch (driveMode) {
//            case 1:


                leftStickYVal = gamepad1.left_stick_y;
                leftStickYVal = Range.clip(leftStickYVal, -1, 1);
                leftStickXVal = gamepad1.left_stick_x;
                leftStickXVal = Range.clip(leftStickXVal, -1, 1);
                rightStickXVal = gamepad1.right_stick_x;
                rightStickXVal = Range.clip(rightStickXVal, -1, 1);

                frontLeftSpeed = (leftStickYVal + leftStickXVal + rightStickXVal);
                frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

                frontRightSpeed = (leftStickYVal - leftStickXVal - rightStickXVal);
                frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

                rearLeftSpeed = (leftStickYVal - leftStickXVal + rightStickXVal);
                rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

                rearRightSpeed = (leftStickYVal + leftStickXVal - rightStickXVal);
                rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                    frontLeftSpeed = 0;
                    drive.frontLeftMotor.setPower(frontLeftSpeed);
                } else {
                    drive.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
                }

                if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
                    frontRightSpeed = 0;
                    drive.frontRightMotor.setPower(frontRightSpeed);
                } else {
                    drive.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
                }

                if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                    rearLeftSpeed = 0;
                    drive.backLeftMotor.setPower(rearLeftSpeed);
                } else {
                    drive.backLeftMotor.setPower(rearLeftSpeed * speedMultiply);
                }

                if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
                    rearRightSpeed = 0;
                    drive.backRightMotor.setPower(rearRightSpeed);
                } else {
                    drive.backRightMotor.setPower(rearRightSpeed * speedMultiply);
                }

//                break;
//            case 2:
//
////                leftStickYVal = gamepad1.left_stick_y;
////                leftStickYVal = Range.clip(leftStickYVal, -1, 1);
////
////                rightStickYVal = gamepad1.right_stick_y;
////                rightStickYVal = Range.clip(rightStickYVal, -1, 1);
////
////                leftSidePower = speedMultiply * leftStickYVal * (1);
////                rightSidePower = speedMultiply * rightStickYVal * (1);
////                drive.tankDrive(leftSidePower, rightSidePower);
//                break;

       // }
    }

    public void telemetry(){
        telemetry.addData("fl power", drive.frontLeftMotor.getPower());
        telemetry.addData("fr Power", drive.frontRightMotor.getPower());
        telemetry.addData("rl Power", drive.backLeftMotor.getPower());
        telemetry.addData("rr Power", drive.backRightMotor.getPower());
        telemetry.addData("speed multipy value:",speedMultiply);
        if (driveMode == 1){
            telemetry.addLine("Non Tank");
        }
        else if (driveMode == 2){
            telemetry.addLine("TANK");
        }
        telemetry.update();

    }


    public void switchDriveMode(){
//        if (gamepad1.a){
//            driveMode = 1;
//        }
//        else if (gamepad1.b){
//            driveMode = 2;
//        }
    }
}
