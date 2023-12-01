package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FTCLib Field-Centric Drive (Motor Groups)", group="Testing")
public class DriveMotorGroups extends LinearOpMode {
    private static final double SLIDER_SENSITIVITY = 20;
    private static final double MAX_SLIDER = 12000;
    private static final double SERVO_INCREMENT = 1;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(
                new MotorEx(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_435)
        );

        MotorEx motorSliderLeft = new MotorEx(hardwareMap, "motorSliderLeft");
        MotorEx motorSliderRight = new MotorEx(hardwareMap, "motorSliderRight");

        motorSliderRight.setInverted(true);

        MotorGroup motorSliders = new MotorGroup(motorSliderLeft, motorSliderRight);

        motorSliders.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorSliders.setRunMode(Motor.RunMode.PositionControl);
        motorSliders.stopAndResetEncoder();

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

        double clawAngle = 0, sliderPos = 0;

        waitForStart();

        servoIntakeLeft.turnToAngle(74);
        servoIntakeRight.turnToAngle(77);

        servoBaseRight.turnToAngle(0);

        while (!isStopRequested()) {
            motorIntake.set(-driverOp.getRightY() / 2);

//            Thread launchDrone = new Thread(() -> {
//                servoDrone.turnToAngle(60);
//
//                sleep(2000);
//
//                servoDrone.turnToAngle(0);
//            });
//
//            if (gamepad1.back) launchDrone.start();

            double rT = gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double lT = gunnerOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            sliderPos += SLIDER_SENSITIVITY * ((sliderPos < MAX_SLIDER ? rT : 0) - (sliderPos > 0 ? lT : 0));
            motorSliders.setTargetPosition((int) (sliderPos));
            motorSliders.set(rT - lT);

            servoPitch.rotateByAngle(SERVO_INCREMENT * gunnerOp.getRightY());

            double baseAngleIncrement = SERVO_INCREMENT * gunnerOp.getLeftY();
            servoBaseLeft.rotateByAngle(baseAngleIncrement);
            servoBaseRight.rotateByAngle(baseAngleIncrement);

            double intakeAngleIncrement = driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) ? SERVO_INCREMENT : (driverOp.getButton(GamepadKeys.Button.DPAD_UP) ? -SERVO_INCREMENT : 0);
            servoIntakeLeft.rotateByAngle(intakeAngleIncrement);
            servoIntakeRight.rotateByAngle(intakeAngleIncrement);

            clawAngle = gunnerOp.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 45 : (gunnerOp.getButton(GamepadKeys.Button.DPAD_LEFT) ? 0 : clawAngle);
            servoClaw.turnToAngle(clawAngle);

            drive.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX());

            // region Telemetry Logging
            telemetry.addData("sliderPos", sliderPos);
            telemetry.addData("Slider Power", motorSliders.get());
            telemetry.addData("Slider Current Pos", motorSliderLeft.getCurrentPosition());
            telemetry.addData("Base Left", servoBaseLeft.getAngle());
            telemetry.addData("Base Right", servoBaseRight.getAngle());
            telemetry.addData("Intake Left", servoIntakeLeft.getAngle());
            telemetry.addData("Intake Right", servoIntakeRight.getAngle());
            telemetry.addData("Pitch", servoPitch.getAngle());
            telemetry.addData("Claw", servoClaw.getAngle());
            telemetry.update();
            // endregion
        }
    }
}
