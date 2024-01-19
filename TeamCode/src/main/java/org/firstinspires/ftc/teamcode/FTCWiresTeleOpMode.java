package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * FTC WIRES TeleOp Example
 *
 */


@TeleOp(name = "FTC Wires TeleOp", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {

//    DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
//    DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
//    DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
//    DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
    DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");
    DcMotor armMotor = hardwareMap.dcMotor.get("armer");
    DcMotor armMotor2 = hardwareMap.dcMotor.get("armer2");
    Servo planeServo = hardwareMap.servo.get("droneLauncher");

    @Override
    public void runOpMode() throws InterruptedException {
//        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort // unnecessary for us
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","TEAM NUMBER");
        telemetry.update();

        double slowMode = gamepad1.left_trigger;
        double slowCoeff = 0.3;

        double lifterThing = -gamepad2.right_stick_y;
        double operatorArm = -gamepad2.left_stick_y;
        boolean plane = false;


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {

                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","TEAM NUMBER");

                if (slowMode > 0.2) { // check if we are trying to enter "slowmode" with the left trigger on driver gamepad
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y * slowCoeff,
                                    -gamepad1.left_stick_x * slowCoeff
                            ),
                            -gamepad1.right_stick_x * slowCoeff
                    ));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                }

                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor.setPower(operatorArm * 0.4);
                armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor2.setPower(operatorArm * 0.4);
                lifterMotor.setPower(lifterThing);

                if (gamepad2.right_bumper && gamepad2.left_bumper) {
                    plane = true;
                } else {
                    plane = false;
                }

                if (plane == true) {
                    planeServo.setPosition(-0.25);
                } else {
                    planeServo.setPosition(0.25);
                }

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