package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp (group = "CompTELEOP",name = "spooltesting")
public class testing_Motor_Speed extends LinearOpMode {
    private DcMotorEx leftspool = null;
    private DcMotorEx rightspool = null;
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");

        leftspool = hardwareMap.get(DcMotorEx.class, "leftspool");
        rightspool = hardwareMap.get(DcMotorEx.class, "rightspool");
// add motors as required


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
           leftspool.setPower(gamepad1.left_stick_y);
           rightspool.setPower(-gamepad1.left_stick_y);

           telemetry.addData("leftspoolpos",leftspool.getCurrentPosition());
           telemetry.addData("rightspoolpos", rightspool.getCurrentPosition());






        }
    }
}