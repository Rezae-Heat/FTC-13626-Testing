package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="Autonomous Park Blue", group="Autonomous Programs")
public class AutonomousParkBlue extends LinearOpMode {
    private final double DISTANCE_PER_TICK = Math.PI * 0.096 / 537.7;
    private final double DISTANCE = 3 / 3.281;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor fL = new Motor(hardwareMap, "motorFrontLeft", Motor.GoBILDA.RPM_435);
        Motor fR = new Motor(hardwareMap, "motorFrontRight", Motor.GoBILDA.RPM_435);
        Motor bL = new Motor(hardwareMap, "motorBackLeft", Motor.GoBILDA.RPM_435);
        Motor bR = new Motor(hardwareMap, "motorBackRight", Motor.GoBILDA.RPM_435);

        fL.setDistancePerPulse(DISTANCE_PER_TICK);
        fR.setDistancePerPulse(DISTANCE_PER_TICK);
        bL.setDistancePerPulse(DISTANCE_PER_TICK);
        bR.setDistancePerPulse(DISTANCE_PER_TICK);

        fL.setTargetDistance(-DISTANCE);
        fR.setTargetDistance(DISTANCE);
        bL.setTargetDistance(DISTANCE);
        bR.setTargetDistance(-DISTANCE);

        waitForStart();

        while (!isStopRequested()) {
            if (fL.encoder.getDistance() != -DISTANCE || bL.encoder.getDistance() != DISTANCE) {
                fL.set(0.5);
                fR.set(0.5);
                bL.set(0.5);
                bR.set(0.5);
                continue;
            }

            fL.stopMotor();
            fR.stopMotor();
            bL.stopMotor();
            bR.stopMotor();
        }
    }
}