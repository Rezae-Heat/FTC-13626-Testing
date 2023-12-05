package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name="Graduation", group="Driver Control")
public class MyBeautifulDarkTwistedFantasy extends LinearOpMode {
    private static final ExecutorService threadPool = Executors.newFixedThreadPool(3);

    private static final double SLIDER_VELOCITY = 2000;
    public static final int SLIDER_HIGH = 770;
    public static final int SLIDER_MID = 450;
    public static final int SLIDER_LOW = 200;

    private static final double INTAKE_OFFSET = -3;

    private static boolean presetActive;

    @Override
    public void runOpMode() {
        MotorEx fL = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        MotorEx fR = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        MotorEx bL = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);
        MotorEx bR = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);

        MecanumDrive driveTrain = new MecanumDrive(fL, fR, bL, bR);

        DcMotorEx motorSliderLeft = hardwareMap.get(DcMotorEx.class, "leftSpool");
        DcMotorEx motorSliderRight = hardwareMap.get(DcMotorEx.class, "rightSpool");

        motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSliderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Motor motorIntake = new Motor(hardwareMap, "intake");

        ServoEx servoClaw = new SimpleServo(hardwareMap, "claw", 0, 255);
        ServoEx servoPitch = new SimpleServo(hardwareMap, "wrist", 0, 180);
        ServoEx servoLatch = new SimpleServo(hardwareMap, "latch", 0, 180);
        ServoEx servoDrone = new SimpleServo(hardwareMap, "drone", 0, 60);
        ServoEx servoBaseLeft = new SimpleServo(hardwareMap, "leftElbow", 0, 255);
        ServoEx servoBaseRight = new SimpleServo(hardwareMap, "rightElbow", 0, 255);
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "leftIntake", 0, 180);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "rightIntake", 0, 180);

        servoClaw.setInverted(true);
        servoPitch.setInverted(true);
        servoBaseLeft.setInverted(true);
        servoBaseRight.setInverted(true);
        servoIntakeRight.setInverted(true);

        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        servoIntakeLeft.turnToAngle(77);
        servoIntakeRight.turnToAngle(servoIntakeLeft.getAngle() + INTAKE_OFFSET);

        servoBaseRight.turnToAngle(0);
        servoClaw.turnToAngle(205);
        servoLatch.turnToAngle(180);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Runnable readyGrab = () -> {
                presetActive = true;

                servoIntakeLeft.turnToAngle(74);
                servoIntakeRight.turnToAngle(servoIntakeLeft.getAngle() + INTAKE_OFFSET);

                servoClaw.turnToAngle(180);
                servoBaseLeft.turnToAngle(90);
                servoPitch.turnToAngle(190);

                sleep(500);

                servoPitch.turnToAngle(220);
                servoLatch.turnToAngle(180);

                motorSliderLeft.setTargetPosition(-10);
                motorSliderRight.setTargetPosition(-10);

                motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                presetActive = false;
            };

            Runnable grabPixelLow = () -> {
                presetActive = true;

                servoLatch.turnToAngle(0);
                servoBaseLeft.turnToAngle(45);
                servoPitch.turnToAngle(200);

                sleep(1000);

                servoClaw.turnToAngle(0);

                sleep(1000);

                servoBaseLeft.turnToAngle(90);

                sleep(200);

                servoPitch.turnToAngle(230);
                servoBaseLeft.turnToAngle(90);

                sleep(800);

                motorSliderLeft.setTargetPosition(SLIDER_LOW);
                motorSliderRight.setTargetPosition(SLIDER_LOW);

                motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                sleep(500);

                servoPitch.turnToAngle(90);
                servoBaseLeft.turnToAngle(175);

                presetActive = false;
            };

            Runnable grabPixelMid = () -> {
                presetActive = true;

                servoLatch.turnToAngle(0);
                servoBaseLeft.turnToAngle(45);
                servoPitch.turnToAngle(200);

                sleep(1000);

                servoClaw.turnToAngle(0);

                sleep(1000);

                servoBaseLeft.turnToAngle(90);

                sleep(200);

                servoPitch.turnToAngle(230);
                servoBaseLeft.turnToAngle(90);

                sleep(800);

                motorSliderLeft.setTargetPosition(SLIDER_MID);
                motorSliderRight.setTargetPosition(SLIDER_MID);

                motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                sleep(500);

                servoPitch.turnToAngle(90);
                servoBaseLeft.turnToAngle(175);

                presetActive = false;
            };

            Runnable grabPixelHigh = () -> {
                presetActive = true;

                servoLatch.turnToAngle(0);
                servoBaseLeft.turnToAngle(45);
                servoPitch.turnToAngle(200);

                sleep(1000);

                servoClaw.turnToAngle(0);

                sleep(1000);

                servoBaseLeft.turnToAngle(90);

                sleep(200);

                servoPitch.turnToAngle(230);
                servoBaseLeft.turnToAngle(90);

                sleep(800);

                motorSliderLeft.setTargetPosition(SLIDER_HIGH);
                motorSliderRight.setTargetPosition(SLIDER_HIGH);

                motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                sleep(500);

                servoPitch.turnToAngle(90);
                servoBaseLeft.turnToAngle(175);

                presetActive = false;
            };

            Runnable launchDrone = () -> {
                servoDrone.turnToAngle(60);
                sleep(2000);
                servoDrone.turnToAngle(0);
            };

            if (gamepad1.back) threadPool.submit(launchDrone);

            if (!presetActive) {
                if (driverOp.getButton(GamepadKeys.Button.A)) {
                    threadPool.submit(readyGrab);
                } else if (driverOp.getButton(GamepadKeys.Button.X)) {
                    threadPool.submit(grabPixelLow);
                } else if (driverOp.getButton(GamepadKeys.Button.Y)) {
                    threadPool.submit(grabPixelMid);
                } else if (driverOp.getButton(GamepadKeys.Button.B)) {
                    threadPool.submit(grabPixelHigh);
                } else {
                    if (driverOp.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                        servoClaw.turnToAngle(150);
                    } else if (driverOp.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                        servoClaw.turnToAngle(0);
                    }
                }
            }

            motorIntake.set(-driverOp.getRightY() / 2);

            driveTrain.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX());

            // region Telemetry Logging
            telemetry.addData("Slider Target Pos", motorSliderLeft.getTargetPosition());
            telemetry.addData("Slider Current Pos", motorSliderLeft.getCurrentPosition());
            telemetry.addData("Slider Velocity", motorSliderLeft.getVelocity());

            String[] servoNames = {"Base Left", "Base Right", "\nIntake Left", "Intake Right", "\nPitch", "Claw"};
            ServoEx[] servos = {servoBaseLeft, servoBaseRight, servoIntakeLeft, servoIntakeRight, servoPitch, servoClaw};

            for (int i = 0; i < servoNames.length; i++) { telemetry.addData(servoNames[i], servos[i].getAngle()); }

            telemetry.update();
            // endregion
        }

        threadPool.shutdown();
    }
}
