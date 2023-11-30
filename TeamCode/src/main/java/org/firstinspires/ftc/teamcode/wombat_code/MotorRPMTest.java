package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor RPM Test", group="Utility")
public class MotorRPMTest extends LinearOpMode {
    DcMotor motorFrontLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            motorFrontLeft.setPower(1);
        }
    }
}
