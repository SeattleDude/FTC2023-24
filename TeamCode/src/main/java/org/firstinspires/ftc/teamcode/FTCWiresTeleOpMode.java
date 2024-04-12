package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * FTC WIRES TeleOp Example
 *
 */



@TeleOp(name = "FTC Wires TeleOp", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {
    DcMotor leftExtend;
    DcMotor rightExtend;
    DcMotor leftRotate;
    DcMotor rightRotate;

    // these are coefficients for tuning the PID controllers for the arm extention and rotation
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    public void PIDPower(double refrence, double position) {

    }

    //Method to rotate Arms

    public void rotateArm(float rotationAmount) {
//        leftRotate.setTargetPosition(rotationAmount);
//        rightRotate.setTargetPosition(-rotationAmount);
        if ((leftRotate.getCurrentPosition() >= 0) || (leftRotate.getCurrentPosition() < 10000)) {
            leftRotate.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            leftRotate.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRotate.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        leftRotate.setPower(rotationAmount);
        rightRotate.setPower(rotationAmount);
    }
    //Method to Extend Arms

    public void extendArm(float extensionAmount) {
//        leftExtend.setTargetPosition(extensionAmount);
//        rightExtend.setTargetPosition(-extensionAmount);
        if ((leftExtend.getCurrentPosition() >= 0) || (leftExtend.getCurrentPosition() < 2000)) {
            leftExtend.setDirection(DcMotorSimple.Direction.FORWARD);
            rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            leftExtend.setDirection(DcMotorSimple.Direction.REVERSE);
            rightExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        leftExtend.setPower(extensionAmount);
        rightExtend.setPower(extensionAmount);
    }

//    DcMotor leftFrontDrive = hardwareMap.dcMotor.get("FLdrive");
//    DcMotor leftBackDrive = hardwareMap.dcMotor.get("BLdrive");
//    DcMotor rightFrontDrive = hardwareMap.dcMotor.get("FRdrive");
//    DcMotor rightBackDrive = hardwareMap.dcMotor.get("BRdrive");

    @Override
    public void runOpMode() throws InterruptedException {

        leftExtend = hardwareMap.get(DcMotor.class, "leftExtend");
        leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtend = hardwareMap.get(DcMotor.class, "rightExtend");
        rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRotate = hardwareMap.get(DcMotor.class, "leftRotate");
        leftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotate = hardwareMap.get(DcMotor.class, "rightRotate");
        rightRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rightExtend.setDirection(DcMotorSimple.Direction.REVERSE);


        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo leftFingerServo = hardwareMap.servo.get("leftFinger");
        Servo rightFingerServo = hardwareMap.servo.get("rightFinger");

        Servo planeServo = hardwareMap.servo.get("droneRelease");

//        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort // unnecessary for us
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","TEAM NUMBER");
        telemetry.update();

        boolean leftGrab = false;
        boolean rightGrab = false;


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // all this needs to be replaced with a custom pid tuned motion method
            leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            11192 extend limit
//            2035 rotate limit

            waitForStart();

            while (opModeIsActive()) {

                boolean plane = false;
                double slowMode = gamepad1.left_trigger;
                double slowCoeff = 0.3;


                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","TEAM NUMBER");

                if (slowMode > 0.2) { // check if we are trying to enter "slowmode" with the left trigger on driver gamepad
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    gamepad1.left_stick_y * slowCoeff,
                                    gamepad1.left_stick_x * slowCoeff
                            ),
                            gamepad1.right_stick_x * slowCoeff
                    ));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    gamepad1.left_stick_y,
                                    gamepad1.left_stick_x
                            ),
                            gamepad1.right_stick_x
                    ));
                }

                if (gamepad2.left_trigger > 0.2) { // rotate CCW
                    wristServo.setPosition(wristServo.getPosition() + 0.02);
                } else if (gamepad2.right_trigger > 0.2) { // rotate CW
                    wristServo.setPosition(wristServo.getPosition() - 0.02);
                }

                if (gamepad2.right_bumper) { // swapped as per Max's suggestion
                    leftGrab = !leftGrab;
                }

                if (gamepad2.left_bumper) {
                    rightGrab = !rightGrab;
                }

                if (leftGrab) {
                    leftFingerServo.setPosition(1);
                } else {
                    leftFingerServo.setPosition(0);
                }

                if (rightGrab) {
                    rightFingerServo.setPosition(0);
                } else {
                    rightFingerServo.setPosition(1);
                }

                if (gamepad1.left_bumper && gamepad1.right_bumper) { // plane lanuch code, this should work like the first time we programmed the claw fingers so the bumpers must be held to fully launch the drone
                    plane = true;
                }

                extendArm(gamepad2.left_stick_x);
                rotateArm(-gamepad2.left_stick_y);

                telemetry.addData("extend encoder ", leftExtend.getCurrentPosition());
                telemetry.addData("rotate encoder ", leftRotate.getCurrentPosition());



//
//                if (gamepad2.right_bumper && gamepad2.left_bumper) {
//                    plane = true;
//                } else {
//                    plane = false;
//                }
//
//                if (plane == true) {
//                    planeServo.setPosition(-0.25);
//                } else {
//                    planeServo.setPosition(0.25);
//                }

                drive.updatePoseEstimate();

                //telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                //telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                //telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                //telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) { // This should NEVER run on our robot, so I commented it all out
//            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//            waitForStart();
//
//            while (opModeIsActive()) {
//                drive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
//                                0.0
//                        ),
//                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
//                ));
//
//                drive.updatePoseEstimate();
//
//                telemetry.addData("x", drive.pose.position.x);
//                telemetry.addData("y", drive.pose.position.y);
//                telemetry.addData("heading", drive.pose.heading);
//                telemetry.update();
//            }
        } else {
            throw new AssertionError();
        }
    }
}