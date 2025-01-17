package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name="Slider Velocity Control", group="Testing")
public class VelocitySliderControl extends LinearOpMode {
    private static final double SLIDER_SENSITIVITY = 20;
    private static final double MAX_SLIDER = 12000;
    private static final double SERVO_INCREMENT = 1;
    private static boolean presetActive, slidersMoving;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(
                new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_435)
        );

        DcMotorEx motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorSliderLeft");
        DcMotorEx motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorSliderRight");

        motorSliderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motor motorIntake = new Motor(hardwareMap, "motorIntake");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ServoEx servoClaw = new SimpleServo(hardwareMap, "servoClaw", 0, 45);
        ServoEx servoPitch = new SimpleServo(hardwareMap, "servoPitch", 0, 180);
//        ServoEx servoDrone = new SimpleServo(hardwareMap, "servoDrone", 0, 60);
        ServoEx servoBaseLeft = new SimpleServo(hardwareMap, "servoBaseLeft", 0, 255);
        ServoEx servoBaseRight = new SimpleServo(hardwareMap, "servoBaseRight", 0, 255);
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "servoIntakeLeft", 0, 180);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "servoIntakeRight", 0, 180);

        servoBaseRight.setInverted(true);
        servoIntakeRight.setInverted(true);

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx gunnerOp = new GamepadEx(gamepad2);

        double clawAngle = 0;
        double sliderPos = 0;

        waitForStart();

        servoIntakeLeft.turnToAngle(74);
        servoIntakeRight.turnToAngle(77);

        servoBaseRight.turnToAngle(0);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Thread grabPixel = new Thread(() -> {
                presetActive = true;

                servoClaw.turnToAngle(22.5);
                servoPitch.turnToAngle(158.04);

                sleep(500);

                servoBaseLeft.turnToAngle(0);
                servoBaseRight.turnToAngle(0);

                sleep(1000);

                servoClaw.turnToAngle(45);

                sleep(500);

                servoBaseLeft.turnToAngle(90);
                servoBaseRight.turnToAngle(90);

                presetActive = false;
            });

            Thread placePixel = new Thread(() -> {
                presetActive = true;

                servoPitch.turnToAngle(100);

                sleep(500);

                servoBaseLeft.turnToAngle(101.7);
                servoBaseRight.turnToAngle(101.7);

                sleep(500);

                servoPitch.turnToAngle(62.1);

                presetActive = false;
            });

            Thread zeroArm = new Thread(() -> {
                presetActive = true;

                servoPitch.turnToAngle(100);

                sleep(500);

                servoBaseLeft.setPosition(90);
                servoBaseRight.setPosition(90);

                sleep(500);

                servoPitch.turnToAngle(158.04);

                slidersMoving = true;

                motorSliderLeft.setTargetPosition(0);
                motorSliderRight.setTargetPosition(0);

                motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (motorSliderLeft.getCurrentPosition() != 0 || motorSliderRight.getCurrentPosition() != 0) {
                    motorSliderLeft.setVelocity(435);
                    motorSliderLeft.setVelocity(435);
                }

                motorSliderLeft.setVelocity(0);
                motorSliderLeft.setVelocity(0);

                slidersMoving = presetActive = false;
            });

//            Thread launchDrone = new Thread(() -> {
//                servoDrone.turnToAngle(60);
//
//                sleep(2000);
//
//                servoDrone.turnToAngle(0);
//            });
//
//            if (gamepad1.back) launchDrone.start();

            if (!slidersMoving) {
                double rT = gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                double lT = gunnerOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                sliderPos += SLIDER_SENSITIVITY * ((sliderPos < MAX_SLIDER ? rT : 0) - (sliderPos > 0 ? lT : 0));

                motorSliderLeft.setTargetPosition((int) (sliderPos));
                motorSliderRight.setTargetPosition((int) (sliderPos));

                double sliderVelocity = 435 * Math.abs(rT - lT);
                motorSliderLeft.setVelocity(sliderVelocity);
                motorSliderLeft.setVelocity(sliderVelocity);
            }

            if (!presetActive) {
                if (gunnerOp.getButton(GamepadKeys.Button.A)) {
                    grabPixel.start(); // Picks up pixel
                } else if (gunnerOp.getButton(GamepadKeys.Button.Y)) {
                    placePixel.start(); // Moves arm to placement position, NOTE: Does not actually release pixel that's up to the driver as they need to adjust the height of the sliders and whatnot accordingly.
                } else if (gunnerOp.getButton(GamepadKeys.Button.B)) {
                    sliderPos = 0;
                    zeroArm.start(); // Moves arm back to neutral position and lowers sliders to zero.
                } else {
                    servoPitch.rotateByAngle(SERVO_INCREMENT * gunnerOp.getRightY());

                    double baseAngleIncrement = SERVO_INCREMENT * gunnerOp.getLeftY();
                    servoBaseLeft.rotateByAngle(baseAngleIncrement);
                    servoBaseRight.rotateByAngle(baseAngleIncrement);

                    double intakeAngleIncrement = driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) ? SERVO_INCREMENT : (driverOp.getButton(GamepadKeys.Button.DPAD_UP) ? -SERVO_INCREMENT : 0);
                    servoIntakeLeft.rotateByAngle(intakeAngleIncrement);
                    servoIntakeRight.rotateByAngle(intakeAngleIncrement);

                    clawAngle = gunnerOp.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 45 : (gunnerOp.getButton(GamepadKeys.Button.DPAD_LEFT) ? 0 : clawAngle);
                    servoClaw.turnToAngle(clawAngle);
                }
            }

            motorIntake.set(-driverOp.getRightY() / 2);

            drive.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX());

            // region Telemetry Logging
            telemetry.addData("Slider Target Pos", sliderPos);
            telemetry.addData("Slider Current Pos", motorSliderLeft.getCurrentPosition());
            telemetry.addData("Slider Velocity", motorSliderLeft.getVelocity());
            telemetry.addData("\nBase Left", servoBaseLeft.getAngle());
            telemetry.addData("Base Right", servoBaseRight.getAngle());
            telemetry.addData("\nIntake Left", servoIntakeLeft.getAngle());
            telemetry.addData("Intake Right", servoIntakeRight.getAngle());
            telemetry.addData("\nPitch", servoPitch.getAngle());
            telemetry.addData("Claw", servoClaw.getAngle());
            telemetry.update();
            // endregion
        }
    }
}
