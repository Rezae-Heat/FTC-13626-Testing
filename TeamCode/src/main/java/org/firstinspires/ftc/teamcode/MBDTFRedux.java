package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="NATIONALS TELEOP REDUX", group="Driver Control")
public class MBDTFRedux extends LinearOpMode {
    private static final double SLIDER_VELOCITY = 2000;
    private static final int SLIDER_LOW = 200, SLIDER_MID = 450, SLIDER_HIGH = 770;

    private static final double INTAKE_OFFSET = 1;
    private static final double LATCH_CLOSED = 10, LATCH_OPEN = 90 + LATCH_CLOSED;
    private static final double INTAKE_DOWN = 74;

    private static boolean presetActive;

    @Override
    public void runOpMode() {
        MotorEx fL = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        MotorEx fR = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        MotorEx bL = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);
        MotorEx bR = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);

        MecanumDrive driveTrain = new MecanumDrive(fL, fR, bL, bR);

        MotorEx motorSliderLeft = new MotorEx(hardwareMap, "leftspool");
        MotorEx motorSliderRight = new MotorEx(hardwareMap, "rightspool");

        motorSliderLeft.setInverted(true);

        motorSliderLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setRunMode(Motor.RunMode.PositionControl);
        motorSliderRight.setRunMode(Motor.RunMode.PositionControl);

        motorSliderLeft.setPositionTolerance(14);
        motorSliderRight.setPositionTolerance(14);

        MotorEx motorIntake = new MotorEx(hardwareMap, "intake");

        ServoEx servoClaw = new SimpleServo(hardwareMap, "claw", 0, 45);
        ServoEx servoPitch = new SimpleServo(hardwareMap, "wrist", 0, 255);
        ServoEx servoLatch = new SimpleServo(hardwareMap, "latch", 0, 180);
        ServoEx servoDrone = new SimpleServo(hardwareMap, "drone", 0, 60);
        ServoEx servoBaseLeft = new SimpleServo(hardwareMap, "leftElbow", 0, 255);
        ServoEx servoBaseRight = new SimpleServo(hardwareMap, "rightElbow", 0, 255);
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "leftIntake", 0, 180);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "rightIntake", 0, 180);

        servoPitch.setInverted(true);
        servoBaseLeft.setInverted(true);
        servoIntakeRight.setInverted(true);

        CRServo servoHangLeft = new CRServo(hardwareMap, "hangLeft");
        CRServo servoHangRight = new CRServo(hardwareMap, "hangRight");

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx gunnerOp = new GamepadEx(gamepad2);

        servoIntakeLeft.turnToAngle(INTAKE_DOWN);
        servoIntakeRight.turnToAngle(servoIntakeLeft.getAngle() + INTAKE_OFFSET);

        servoClaw.turnToAngle(45);
        servoBaseLeft.turnToAngle(90);
        servoBaseRight.turnToAngle(90);
        servoPitch.turnToAngle(190);

        sleep(500);

        servoPitch.turnToAngle(220);
        servoLatch.turnToAngle(LATCH_CLOSED);

        motorSliderLeft.setTargetPosition(-10);
        motorSliderRight.setTargetPosition(-10);

        motorSliderLeft.setVelocity(SLIDER_VELOCITY);
        motorSliderRight.setVelocity(SLIDER_VELOCITY);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Thread readyGrab = new Thread(() -> {
                presetActive = true;

                servoIntakeLeft.turnToAngle(INTAKE_DOWN);
                servoIntakeRight.turnToAngle(servoIntakeLeft.getAngle() + INTAKE_OFFSET);

                servoClaw.turnToAngle(0);
                servoBaseLeft.turnToAngle(90);
                servoBaseRight.turnToAngle(90);
                servoPitch.turnToAngle(190);

                sleep(500);

                servoPitch.turnToAngle(220);
                servoLatch.turnToAngle(LATCH_CLOSED);

                motorSliderLeft.setTargetPosition(-10);
                motorSliderRight.setTargetPosition(-10);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                presetActive = false;
            });

            Thread grabPixelLow = new Thread(() -> {
                presetActive = true;

                servoLatch.turnToAngle(LATCH_OPEN);
                servoBaseLeft.turnToAngle(45);
                servoBaseRight.turnToAngle(45);
                servoPitch.turnToAngle(200);

                sleep(1000);

                servoClaw.turnToAngle(45);

                sleep(1000);

                servoBaseLeft.turnToAngle(90);
                servoBaseRight.turnToAngle(90);

                sleep(200);

                servoPitch.turnToAngle(230);

                sleep(800);

                motorSliderLeft.setTargetPosition(SLIDER_LOW);
                motorSliderRight.setTargetPosition(SLIDER_LOW);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                sleep(500);

                servoPitch.turnToAngle(90);
                servoBaseLeft.turnToAngle(175);
                servoBaseRight.turnToAngle(175);

                presetActive = false;
            });

            Thread grabPixelMid = new Thread(() -> {
                presetActive = true;

                servoLatch.turnToAngle(LATCH_OPEN);
                servoBaseLeft.turnToAngle(45);
                servoBaseRight.turnToAngle(45);
                servoPitch.turnToAngle(200);

                sleep(1000);

                servoClaw.turnToAngle(45);

                sleep(1000);

                servoBaseLeft.turnToAngle(90);
                servoBaseRight.turnToAngle(90);

                sleep(200);

                servoPitch.turnToAngle(230);

                sleep(800);

                motorSliderLeft.setTargetPosition(SLIDER_MID);
                motorSliderRight.setTargetPosition(SLIDER_MID);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                sleep(500);

                servoPitch.turnToAngle(90);
                servoBaseLeft.turnToAngle(175);
                servoBaseRight.turnToAngle(175);

                presetActive = false;
            });

            Thread grabPixelHigh = new Thread(() -> {
                presetActive = true;

                servoLatch.turnToAngle(LATCH_OPEN);
                servoBaseLeft.turnToAngle(45);
                servoBaseRight.turnToAngle(45);
                servoPitch.turnToAngle(200);

                sleep(1000);

                servoClaw.turnToAngle(45);

                sleep(1000);

                servoBaseLeft.turnToAngle(90);
                servoBaseRight.turnToAngle(90);

                sleep(200);

                servoPitch.turnToAngle(230);

                sleep(800);

                motorSliderLeft.setTargetPosition(SLIDER_HIGH);
                motorSliderRight.setTargetPosition(SLIDER_HIGH);

                motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                motorSliderRight.setVelocity(SLIDER_VELOCITY);

                sleep(500);

                servoPitch.turnToAngle(90);
                servoBaseLeft.turnToAngle(175);
                servoBaseRight.turnToAngle(175);

                presetActive = false;
            });

            Thread launchDrone = new Thread(() -> {
                servoDrone.turnToAngle(60);

                sleep(1000);

                servoDrone.turnToAngle(0);
            });

            if (gamepad1.back) launchDrone.start();

            if (!presetActive) {
                if (driverOp.getButton(GamepadKeys.Button.A)) {
                    readyGrab.start();
                } else if (driverOp.getButton(GamepadKeys.Button.X)) {
                    grabPixelLow.start();
                } else if (driverOp.getButton(GamepadKeys.Button.Y)) {
                    grabPixelMid.start();
                } else if (driverOp.getButton(GamepadKeys.Button.B)) {
                    grabPixelHigh.start();
                } else if (driverOp.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                    servoClaw.turnToAngle(0);
                } else if (driverOp.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                    servoClaw.turnToAngle(45);
                }
            }

            double hangPower = gunnerOp.getRightY();
            servoHangLeft.set(hangPower);
            servoHangRight.set(hangPower);

            motorIntake.set(-driverOp.getRightY() / 2);

            driveTrain.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX());

            // region Telemetry Logging
            telemetry.addData("Slider Velocity", motorSliderLeft.getVelocity());
            telemetry.addData("Slider Current Pos", motorSliderLeft.getCurrentPosition());
            telemetry.addData("Slider Reached Target Pos?", motorSliderLeft.atTargetPosition());

            String[] servoNames = {"Base Left", "Base Right", "\nIntake Left", "Intake Right", "\nPitch", "Claw"};
            ServoEx[] servos = {servoBaseLeft, servoBaseRight, servoIntakeLeft, servoIntakeRight, servoPitch, servoClaw};
            for (int i = 0; i < servoNames.length; i++) { telemetry.addData(servoNames[i], servos[i].getAngle());}

            telemetry.update();
            // endregion
        }
    }
}
