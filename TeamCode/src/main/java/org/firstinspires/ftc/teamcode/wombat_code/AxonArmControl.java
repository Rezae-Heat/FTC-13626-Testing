package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Axon Arm Control", group="Utility")
public class AxonArmControl extends LinearOpMode {
    Servo servoClaw;
    Servo servoPitch;

    private double clawPos = 0;
    private double pitchPos = 0;

    @Override
    public void runOpMode() {
        servoClaw = hardwareMap.servo.get("servoClaw");
        servoPitch = hardwareMap.servo.get("servoPitch");

        servoPitch.setDirection(Servo.Direction.REVERSE);
        servoClaw.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            final double SERVO_INCREMENT = 0.0005, MAX_PITCH = 1, MAX_CLAW = 0.25;

            pitchPos += (gamepad2.right_stick_y > 0.1 && pitchPos < MAX_PITCH) ? SERVO_INCREMENT : ((gamepad2.right_stick_y < -0.1 && pitchPos > 0) ? -SERVO_INCREMENT : 0);
            clawPos = (gamepad2.a) ? MAX_CLAW : ((gamepad2.b) ? 0 : clawPos);

            servoClaw.setPosition(clawPos);
            servoPitch.setPosition(pitchPos);

            telemetry.addData("Pitch", pitchPos);
            telemetry.addData("Claw", clawPos);

            telemetry.update();
        }
    }
}
