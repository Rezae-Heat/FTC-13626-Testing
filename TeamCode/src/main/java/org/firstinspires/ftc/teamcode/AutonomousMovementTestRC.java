package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Autonomous Movement Test", group="Testing")
public class AutonomousMovementTestRC extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double SLIDER_VELOCITY = 2000;
    private static final int SLIDER_LOW = 200;

    private static final double LATCH_CLOSED = 10, LATCH_OPEN = 90 + LATCH_CLOSED;

    private static final double INTAKE_OFFSET = 1;

    private static final double WHEEL_CIRCUMFERENCE = Math.PI * 0.096, TICKS_PER_REVOLUTION = 537.7, ROBOT_RADIUS = 0.20;


    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx fL = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        MotorEx fR = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        MotorEx bL = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);
        MotorEx bR = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        fL.setRunMode(Motor.RunMode.PositionControl);
        fR.setRunMode(Motor.RunMode.PositionControl);
        bL.setRunMode(Motor.RunMode.PositionControl);
        bR.setRunMode(Motor.RunMode.PositionControl);

        fL.setPositionTolerance(10);
        fR.setPositionTolerance(10);
        bL.setPositionTolerance(10);
        bR.setPositionTolerance(10);

        MotorEx motorSliderLeft = new MotorEx(hardwareMap, "leftspool");
        MotorEx motorSliderRight = new MotorEx(hardwareMap, "rightspool");

        motorSliderLeft.setInverted(true);

        motorSliderLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setRunMode(Motor.RunMode.PositionControl);
        motorSliderRight.setRunMode(Motor.RunMode.PositionControl);

        motorSliderLeft.setPositionTolerance(14);
        motorSliderRight.setPositionTolerance(14);

        ServoEx servoClaw = new SimpleServo(hardwareMap, "claw", 0, 45);
        ServoEx servoPitch = new SimpleServo(hardwareMap, "wrist", 0, 255);
        ServoEx servoLatch = new SimpleServo(hardwareMap, "latch", 0, 180);
        ServoEx servoBaseLeft = new SimpleServo(hardwareMap, "leftElbow", 0, 255);
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "leftIntake", 0, 180);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "rightIntake", 0, 180);

        servoPitch.setInverted(true);
        servoBaseLeft.setInverted(true);
        servoIntakeRight.setInverted(true);

        servoIntakeLeft.turnToAngle(74);
        servoIntakeRight.turnToAngle(servoIntakeLeft.getAngle() + INTAKE_OFFSET);

        servoClaw.turnToAngle(45);
        servoBaseLeft.turnToAngle(90);
        servoPitch.turnToAngle(190);

        sleep(500);

        servoPitch.turnToAngle(220);
        servoLatch.turnToAngle(LATCH_CLOSED);

        motorSliderLeft.setTargetPosition(-10);
        motorSliderRight.setTargetPosition(-10);

        motorSliderLeft.setVelocity(SLIDER_VELOCITY);
        motorSliderRight.setVelocity(SLIDER_VELOCITY);

        initDetection();

        int targetPos;

        waitForStart();

        if (isStopRequested()) return;

        Thread telemetryLogging = new Thread(() -> {
            while (true) {
                telemetry.addData("Motor BL Position", bL.getCurrentPosition());
                telemetry.addData("Motor BR Position", bR.getCurrentPosition());
                telemetry.addData("Motor FL Position", fL.getCurrentPosition());
                telemetry.addData("Motor FR Position", fR.getCurrentPosition());

                telemetry.update();
            }
        });
        telemetryLogging.start();

        Thread driveToPos = new Thread(() -> {
            while (!fL.atTargetPosition() || !fR.atTargetPosition() || !bL.atTargetPosition() || !bR.atTargetPosition()) {
                fL.setVelocity(543);
                fR.setVelocity(543);
                bL.setVelocity(543);
                bR.setVelocity(543);
            }

            fL.setVelocity(0);
            fR.setVelocity(0);
            bL.setVelocity(0);
            bR.setVelocity(0);
        });

        targetPos = calculatePos(0.6096);
        fL.setTargetPosition(targetPos);
        fR.setTargetPosition(targetPos);
        bL.setTargetPosition(targetPos);
        bR.setTargetPosition(targetPos);

        driveToPos.start();
        driveToPos.join();

        targetPos = calculateRot(90);
        fL.setTargetPosition(targetPos);
        fR.setTargetPosition(-targetPos);
        bL.setTargetPosition(targetPos);
        bR.setTargetPosition(-targetPos);

        driveToPos.start();
        driveToPos.join();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 1 || detection.id == 2 || detection.id == 3) {
                Thread returnArm = new Thread(() -> {
                    servoClaw.turnToAngle(0);

                    sleep(200);

                    servoBaseLeft.turnToAngle(90);
                    servoPitch.turnToAngle(190);

                    sleep(500);

                    servoPitch.turnToAngle(220);
                    servoLatch.turnToAngle(LATCH_CLOSED);

                    motorSliderLeft.setTargetPosition(-10);
                    motorSliderRight.setTargetPosition(-10);

                    motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                    motorSliderRight.setVelocity(SLIDER_VELOCITY);
                });

                Thread transferLow = new Thread(() -> {
                    servoPitch.turnToAngle(230);

                    sleep(800);

                    motorSliderLeft.setTargetPosition(SLIDER_LOW);
                    motorSliderRight.setTargetPosition(SLIDER_LOW);

                    motorSliderLeft.setVelocity(SLIDER_VELOCITY);
                    motorSliderRight.setVelocity(SLIDER_VELOCITY);

                    sleep(500);

                    servoPitch.turnToAngle(90);
                    servoBaseLeft.turnToAngle(175);
                });

                targetPos = calculatePos(detection.ftcPose.x);
                fL.setTargetPosition(targetPos);
                fR.setTargetPosition(-targetPos);
                bL.setTargetPosition(-targetPos);
                bR.setTargetPosition(targetPos);

                driveToPos.start();
                driveToPos.join();

                targetPos = calculatePos(detection.ftcPose.y - 0.1);
                fL.setTargetPosition(targetPos);
                fR.setTargetPosition(targetPos);
                bL.setTargetPosition(targetPos);
                bR.setTargetPosition(targetPos);

                driveToPos.start();
                driveToPos.join();

                targetPos = calculateRot(180);
                fL.setTargetPosition(-targetPos);
                fR.setTargetPosition(targetPos);
                bL.setTargetPosition(-targetPos);
                bR.setTargetPosition(targetPos);

                driveToPos.start();
                driveToPos.join();

                transferLow.start();
                transferLow.join();

                returnArm.start();
                returnArm.join();

                targetPos = calculatePos(0.05);
                fL.setTargetPosition(targetPos);
                fR.setTargetPosition(targetPos);
                bL.setTargetPosition(targetPos);
                bR.setTargetPosition(targetPos);

                driveToPos.start();
                driveToPos.join();

                targetPos = calculateRot(90);
                fL.setTargetPosition(-targetPos);
                fR.setTargetPosition(targetPos);
                bL.setTargetPosition(-targetPos);
                bR.setTargetPosition(targetPos);

                driveToPos.start();
                driveToPos.join();

                break;
            }
        }
    }

    private int calculatePos(double metres) {
        return (int) (metres / WHEEL_CIRCUMFERENCE * TICKS_PER_REVOLUTION);
    }

    private int calculateRot(double degrees) {
        return calculatePos((degrees * Math.PI/180) * ROBOT_RADIUS);
    }

    private void initDetection() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setLensIntrinsics(1445.262036, 1458.020517, 624.518936, 335.987846)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "FrontalCamera"))
                .setCameraResolution(new Size(1280, 720))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessors(aprilTag)
                .build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("\n# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (m)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (m, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nKey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}