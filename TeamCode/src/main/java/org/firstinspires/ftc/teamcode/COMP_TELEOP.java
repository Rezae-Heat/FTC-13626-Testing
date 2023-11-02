package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class COMP_TELEOP extends OpMode {
    private DcMotorEx leftspool = null;
    private DcMotorEx rightspool = null;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx intake;

    private ServoEx rightElbow;
    private ServoEx leftElbow;
    private ServoEx wrist;
    private ServoEx gripper;
    private ServoImplEx drone;
    private ServoEx latch;


    public static final int MAX_SLIDE_VELOCITY = 2000;

    public static final int SLIDE_HIGH = 2000;
    public static final int SLIDE_MID = 1200;
    public static final int SLIDE_LOW = 400;

@Override
public void init() {

    leftspool = hardwareMap.get(DcMotorEx.class, "leftspool");
    rightspool = hardwareMap.get(DcMotorEx.class, "rightspool");

    frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
    backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
    frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
    backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

    intake = hardwareMap.get(DcMotorEx.class, "rightRear");

    rightspool.setDirection(DcMotorEx.Direction.FORWARD);
    leftspool.setDirection(DcMotorEx.Direction.REVERSE);
    leftspool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightspool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    ServoEx rightElbow = new SimpleServo(
            hardwareMap, "rightElbow", 0, 255);
    rightElbow.setInverted(true);
    ServoEx leftElbow = new SimpleServo(
            hardwareMap, "leftElbow", 0, 255);
    ServoEx wrist = new SimpleServo(
            hardwareMap, "wrist", 0, 270);
    ServoEx gripper = new SimpleServo(
            hardwareMap, "claw", 0, 300);
    ServoEx latch = new SimpleServo(
            hardwareMap, "latch", 0, 270);

    ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
}

@Override
public void loop() {
    //</Slides>
    if(gamepad2.dpad_up) {
        leftspool.setTargetPosition(SLIDE_HIGH);
        rightspool.setTargetPosition(SLIDE_HIGH);

        leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftspool.setVelocity(MAX_SLIDE_VELOCITY);
        rightspool.setVelocity(MAX_SLIDE_VELOCITY);
    } else if (gamepad2.dpad_down) {
        leftspool.setTargetPosition(0);
        rightspool.setTargetPosition(0); // FIXME: i highly recommend adding a separate threaded subsystem handler to prevent the spool motors from self combusting

        leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftspool.setVelocity(MAX_SLIDE_VELOCITY);
        rightspool.setVelocity(MAX_SLIDE_VELOCITY);
    } else {
        leftspool.setVelocity(0);
        rightspool.setVelocity(0);
    }
    
    telemetry.addData("Slide Pos: ", (leftspool.getCurrentPosition() + rightspool.getCurrentPosition())/2);
//</Slides end>



    //<Drive code>
    double y = gamepad1.left_stick_y;// Remember, Y stick value is reversed
    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
    double rx = gamepad1.right_stick_x; //gamepad1.right_stick_x;

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;
    double speedMod =1.25;  // only for testing purposes - not final driver configuration

    frontLeftMotor.setPower(frontLeftPower*speedMod);
    backLeftMotor.setPower(backLeftPower*speedMod);
    frontRightMotor.setPower(frontRightPower*speedMod);
    backRightMotor.setPower(backRightPower*speedMod);
    double totalCurrentDT = frontLeftMotor.getCurrentPosition();

    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//</Drive code>

    // Intake
        intake.setPower(gamepad2.left_stick_y / 2);

        //
    // Drone
    if (gamepad2.right_bumper) {
        drone.setPosition(0.25);
    }
        //
    // latch
        if (gamepad2.x){
        latch.turnToAngle(25);
        }
        if (gamepad2.y){
            latch.turnToAngle(90);
        }

}
@Override
public void stop(){
    leftspool.setMotorDisable();
    rightspool.setMotorDisable();

    frontLeftMotor.setMotorDisable();
    frontRightMotor.setMotorDisable();
    backLeftMotor.setMotorDisable();
    backRightMotor.setMotorDisable();

    }
}
