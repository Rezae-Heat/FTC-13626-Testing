package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp (group = "CompTELEOP",name = "NATIONALS:OPTIMISED_TELEOP")
public class Nationals_TELEOP extends OpMode {
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
    private ServoEx latch; // connected to servo port 4
    private CRServo leftHang;
    private CRServo rightHang;



    public static final int MAX_SLIDE_VELOCITY = 2000;

            ;

    public static final int SLIDE_HIGH = 770;
    public static final int SLIDE_MID = 450;
    public static final int SLIDE_LOW = 200;

    @Override
    public void init_loop() {
        leftspool.setMotorEnable();
        rightspool.setMotorEnable();

        intake.setMotorEnable();

        frontLeftMotor.setMotorEnable();
        frontRightMotor.setMotorEnable();
        backLeftMotor.setMotorEnable();
        backRightMotor.setMotorEnable();
    }

    @Override
    public void init() {

        leftspool = hardwareMap.get(DcMotorEx.class, "leftspool");
        rightspool = hardwareMap.get(DcMotorEx.class, "rightspool");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

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
                hardwareMap, "wrist", 0, 255);
        //wrist = hardwareMap.get(CRServo.class,"wrist");
        //gripper = hardwareMap.get(CRServo.class, "claw");
        ServoEx latch = new SimpleServo(
                hardwareMap, "latch", 0, 270); // latch needs to be plugged in
        ServoEx gripper= new SimpleServo(
                hardwareMap, "claw", 0, 255);
        //latch = hardwareMap.get(CRServo.class,"latch");

        ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
        //telemetry.addLine("Ready to Rock n Roll")
        CRServo leftHang = hardwareMap.crservo.get("lefthang");
        CRServo rightHang = hardwareMap.crservo.get("righthang");



       // latch.turnToAngle(180);
        gripper.setInverted(true);
       // gripper.turnToAngle(205);
       //2 leftElbow.turnToAngle(45);
        //rightElbow.turnToAngle(45);
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "leftIntake", 0, 300);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "rightIntake", 0, 300);
       // servoIntakeLeft.turnToAngle(300);
        //servoIntakeRight.turnToAngle(servoIntakeLeft.getAngle());
        latch.turnToAngle(79.6);

        gripper.turnToAngle(255);
        leftElbow.turnToAngle(60);
    }

    @Override
    public void loop() {

            leftspool = hardwareMap.get(DcMotorEx.class, "leftspool");
            rightspool = hardwareMap.get(DcMotorEx.class, "rightspool");

            frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
            backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
            frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
            backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

            intake = hardwareMap.get(DcMotorEx.class, "intake");
            ServoEx rightElbow = new SimpleServo(
                    hardwareMap, "rightElbow", 0, 255);
            rightElbow.setInverted(false);

            ServoEx leftElbow = new SimpleServo(
                    hardwareMap, "leftElbow", 0, 255);

            ServoEx wrist = new SimpleServo(
                    hardwareMap, "wrist", 0, 270);
            //wrist = hardwareMap.get(CRServo.class,"wrist");
            //gripper = hardwareMap.get(CRServo.class, "claw");
            ServoEx latch = new SimpleServo(
                    hardwareMap, "latch", 0, 270);
            ServoEx gripper= new SimpleServo(
                    hardwareMap, "claw", 0, 255);

        CRServo leftHang = hardwareMap.crservo.get("lefthang");
        CRServo rightHang = hardwareMap.crservo.get("righthang");
            ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
            telemetry.addLine("Ready to Rock n Roll");
        ServoEx servoIntakeLeft = new SimpleServo(hardwareMap, "leftIntake", 0, 300);
        ServoEx servoIntakeRight = new SimpleServo(hardwareMap, "rightIntake", 0, 300);


            //<Drive code>
            double y = -gamepad1.left_stick_y;// Remember, Y stick value is reversed
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
            double speedMod = 1;  // only for testing purposes - not final driver configuration

            frontLeftMotor.setPower(frontLeftPower * speedMod);
            backLeftMotor.setPower(backLeftPower * speedMod);
            frontRightMotor.setPower(frontRightPower * speedMod);
            backRightMotor.setPower(backRightPower * speedMod);
            double totalCurrentDT = frontLeftMotor.getCurrentPosition();

            frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //</Drive code>

            ElapsedTime spacer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            leftElbow.setInverted(false);
            wrist.setInverted(true);
            gripper.setInverted(true);
            telemetry.addData("timer", spacer.time());
            if (gamepad1.right_bumper){
                speedMod =1;
            }
            // Intake run
            intake.setPower(gamepad1.right_stick_y);
                telemetry.addData("rightelbow",rightElbow.getAngle());
                telemetry.addData("leftelbow",leftElbow.getAngle());
                telemetry.addData("rightspool",rightspool.getCurrentPosition());
                telemetry.addData("leftspool",leftspool.getCurrentPosition());
                latch.turnToAngle(77);
              if (gamepad1.a){ // Intake
                  latch.turnToAngle(78);
                gripper.turnToAngle(255);
                leftElbow.turnToAngle(55);
                //rightElbow.turnToAngle(45);

                wrist.turnToAngle(205);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                wrist.turnToAngle(220);
                spacer.reset();


                leftspool.setTargetPosition(0);
                rightspool.setTargetPosition(0); // FIXME: i highly recommend adding a separate threaded subsystem handler to prevent the spool motors from self combusting

                leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftspool.setVelocity(MAX_SLIDE_VELOCITY);
                rightspool.setVelocity(MAX_SLIDE_VELOCITY);
            }
            else if (gamepad1.x) { // transfer - low pos
                    gripper.turnToAngle(250);
                  latch.turnToAngle(0);
                  try {
                      Thread.sleep(500);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  leftElbow.turnToAngle(0);
                  wrist.turnToAngle(255);
                  spacer.startTime();
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  gripper.turnToAngle(0);
                  try {
                      Thread.sleep(500);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }

                  leftElbow.turnToAngle(50);

                  try {
                      Thread.sleep(200);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  wrist.turnToAngle(230);
                  leftElbow.turnToAngle(55); //FIXME: Another thread needs to be called for sleep, otherwise the robot stops
                  try {
                      Thread.sleep(800);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  leftspool.setTargetPosition(SLIDE_LOW);
                  rightspool.setTargetPosition(SLIDE_LOW);

                  leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                  rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                  leftspool.setVelocity(MAX_SLIDE_VELOCITY);
                  rightspool.setVelocity(MAX_SLIDE_VELOCITY);
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }

                  leftElbow.turnToAngle(145);
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  wrist.turnToAngle(150);
                  gripper.turnToAngle(0);
                  spacer.reset();
            } // end of loop
            else if (gamepad1.y) { // transfer

                  gripper.turnToAngle(250);
                  latch.turnToAngle(0);
                  try {
                      Thread.sleep(500);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  leftElbow.turnToAngle(0);
                  wrist.turnToAngle(255);
                  spacer.startTime();
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  gripper.turnToAngle(0);
                  try {
                      Thread.sleep(500);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }

                  leftElbow.turnToAngle(50);

                  try {
                      Thread.sleep(200);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  wrist.turnToAngle(230);
                  leftElbow.turnToAngle(55); //FIXME: Another thread needs to be called for sleep, otherwise the robot stops
                  try {
                      Thread.sleep(800);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  leftspool.setTargetPosition(SLIDE_MID);
                  rightspool.setTargetPosition(SLIDE_MID);

                  leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                  rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                  leftspool.setVelocity(MAX_SLIDE_VELOCITY);
                  rightspool.setVelocity(MAX_SLIDE_VELOCITY);
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }

                  leftElbow.turnToAngle(145);
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  wrist.turnToAngle(150);
                  gripper.turnToAngle(0);
                  spacer.reset();
            } // end of loop
              else if (gamepad1.b) {
                  gripper.turnToAngle(250);
                  latch.turnToAngle(0);
                  try {
                      Thread.sleep(500);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  leftElbow.turnToAngle(0);
                  wrist.turnToAngle(255);
                  spacer.startTime();
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  gripper.turnToAngle(0);
                  try {
                      Thread.sleep(500);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }

                  leftElbow.turnToAngle(50);

                  try {
                      Thread.sleep(200);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  wrist.turnToAngle(230);
                  leftElbow.turnToAngle(55); //FIXME: Another thread needs to be called for sleep, otherwise the robot stops
                  try {
                      Thread.sleep(800);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  leftspool.setTargetPosition(SLIDE_HIGH);
                  rightspool.setTargetPosition(SLIDE_HIGH);

                  leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                  rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                  leftspool.setVelocity(MAX_SLIDE_VELOCITY);
                  rightspool.setVelocity(MAX_SLIDE_VELOCITY);
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }

                  leftElbow.turnToAngle(145);
                  try {
                      Thread.sleep(1000);
                  } catch (InterruptedException e) {
                      throw new RuntimeException(e);
                  }
                  wrist.turnToAngle(150);
                  gripper.turnToAngle(0);
                  spacer.reset();
              }
        leftHang.setPower(gamepad2.left_stick_y);
        rightHang.setPower(gamepad2.right_stick_y);

        if (gamepad1.left_stick_button){ // 255 all the way down
                gripper.turnToAngle(0); // opens gripper


                //leftElbow.turnToAngle(180);
               // wrist.turnToAngle(255); // scoring side

            }
            if (gamepad1.right_stick_button){
              //  wrist.turnToAngle(0); // 0 all the way up
                gripper.turnToAngle(255); // closes gripper

                // leftElbow.turnToAngle(45);


            }
            if (gamepad1.left_bumper){
                speedMod=4;
            }
            if (gamepad1.right_trigger>0.5){
                drone.setPosition(0.25);
            }
            telemetry.addData("leftspoolpos",leftspool.getCurrentPosition());
            telemetry.addData("rightspoolpos",rightspool.getCurrentPosition());
    }


    @Override
    public void stop() {
        leftspool.setMotorDisable();
        rightspool.setMotorDisable();

        intake.setMotorDisable();

        frontLeftMotor.setMotorDisable();
        frontRightMotor.setMotorDisable();
        backLeftMotor.setMotorDisable();
        backRightMotor.setMotorDisable();

    }
}

