package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FTCLib Field-Centric Drive", group="Driver Control")
public class DriverControlFTCLib extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_435),
                new Motor(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_435)
        );

        MotorEx motorSliderLeft = new MotorEx(hardwareMap, "motorSliderLeft");
        MotorEx motorSliderRight = new MotorEx(hardwareMap, "motorSliderRight");

        motorSliderLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        motorSliderLeft.setRunMode(Motor.RunMode.PositionControl);
        motorSliderRight.setRunMode(Motor.RunMode.PositionControl);

        motorSliderRight.setInverted(true);

        Motor motorIntake = new Motor(hardwareMap, "motorIntake");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ServoEx servoPitch = new SimpleServo(hardwareMap, "servoPitch", 0, 180);
        ServoEx servoClaw = new SimpleServo(hardwareMap, "servoClaw", 0, 45);
        ServoEx servoBaseLeft = new SimpleServo(hardwareMap, "servoBaseLeft", 0, 255);
        ServoEx servoBaseRight = new SimpleServo(hardwareMap, "servoBaseRight", 0, 255);
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "servoIntakeLeft", 0, 180);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "servoIntakeRight", 0, 180);

        ServoEx servoDrone = new SimpleServo(hardwareMap, "servoDrone", 0, 60);

        servoBaseRight.setInverted(true);
        servoIntakeRight.setInverted(true);

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx gunnerOp = new GamepadEx(gamepad2);

        double clawAngle = 0, sliderPos = 0;

        waitForStart();

        while (!isStopRequested()) {
            motorIntake.set(gamepad1.x ? 1 : (gamepad1.b ? -1 : 0));

            Thread launchDrone = new Thread(() -> {
                servoDrone.turnToAngle(60);
                sleep(2000);
                servoDrone.turnToAngle(0);
            });
            if (gamepad1.back) launchDrone.start();

            double prevSliderPos = sliderPos;
            sliderPos += 20 * (gunnerOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gunnerOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            int threadSliderPos = (int) (sliderPos);

            Thread moveSliders = new Thread(() -> {
                motorSliderLeft.setTargetPosition(threadSliderPos);
                motorSliderRight.setTargetPosition(threadSliderPos);

                motorSliderLeft.set(0);
                motorSliderRight.set(0);

                motorSliderLeft.setPositionTolerance(14);
                motorSliderRight.setPositionTolerance(14);

                while (!motorSliderLeft.atTargetPosition() || !motorSliderRight.atTargetPosition()) {
                    motorSliderLeft.set(0.8);
                    motorSliderRight.set(0.8);
                }

                motorSliderLeft.stopMotor();
                motorSliderRight.stopMotor();
            });
            if (prevSliderPos != threadSliderPos) moveSliders.start();

            servoPitch.rotateByAngle(0.1 * gunnerOp.getRightY());

            double baseAngleIncrement = 0.1 * gunnerOp.getLeftY();
            servoBaseLeft.rotateByAngle(baseAngleIncrement);
            servoBaseRight.rotateByAngle(baseAngleIncrement);

            double intakeAngleIncrement = driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) ? 0.1 : (driverOp.getButton(GamepadKeys.Button.DPAD_UP) ? -0.1 : 0);
            servoIntakeLeft.rotateByAngle(intakeAngleIncrement);
            servoIntakeRight.rotateByAngle(intakeAngleIncrement);

            clawAngle = gunnerOp.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 45 : (gunnerOp.getButton(GamepadKeys.Button.DPAD_LEFT) ? 0 : clawAngle);
            servoClaw.turnToAngle(clawAngle);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drive.driveFieldCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), heading);

            // region Telemetry Logging
            telemetry.addData("sliderPos", sliderPos);
            telemetry.addData("Slider Left", motorSliderLeft.getCurrentPosition());
            telemetry.addData("Slider Right", motorSliderRight.getCurrentPosition());
            telemetry.addData("Base Left", servoBaseLeft.getAngle());
            telemetry.addData("Base Right", servoBaseRight.getAngle());
            telemetry.addData("Pitch", servoPitch.getAngle());
            telemetry.addData("Claw", servoClaw.getAngle());
            telemetry.update();
            // endregion
        }
    }
}
