package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter"); // upper limit 13599, lower limit is 0
        DcMotor armMotor = hardwareMap.dcMotor.get("armer");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("armer2"); // second motor for enough torque
        Servo planeServo = hardwareMap.servo.get("serv1");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // gotta restart the motor after stopping to zero the encoder
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // same same

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -1 * gamepad1.right_stick_x;

            double slowMode = gamepad1.left_trigger;
            double slowCoeff = 0.3;
            double lifterThing = gamepad2.right_stick_y;
            double operatorArm = -gamepad2.left_stick_y;
            boolean plane = false;
            SpikeDetectionPos Pos = SpikeDetectionPos.UNK;

            int lifterUP = 13500;
            int lifterHang = 4800;
            int lifterDown = 0;
//            int armUP = 0;
//            int armDown = -60;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            int armPos = armMotor.getCurrentPosition();
//            int armPos2 = armMotor2.getCurrentPosition();
            int lifterPos = lifterMotor.getCurrentPosition();


            if (gamepad2.right_bumper && gamepad2.left_bumper) { // double safety so you don't
                                                                 // negligently launch
                plane = true;
            } else {
                plane = false;
            }

            if (plane == true) {
                planeServo.setPosition(-0.25);
            } else {
                planeServo.setPosition(0.25); // might need to be at 0.5 still?
            }

            if (slowMode > 0.2) {
                frontLeftMotor.setPower(frontLeftPower * slowCoeff); // drive motor things
                backLeftMotor.setPower(backLeftPower * slowCoeff);
                frontRightMotor.setPower(frontRightPower * slowCoeff);
                backRightMotor.setPower(backRightPower * slowCoeff);
            } else {
                frontLeftMotor.setPower(frontLeftPower); // drive motor things
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

//            if (armPos >= 50) { // set upper limits for arm so it doesn't smash itself
//                armMotor.setPower(-gamepad2.left_stick_y);
//            }
//
//            if (lifterPos >= 13500) { // set upper limits for lifter so it doesn't strip the lifter nut
//                lifterMotor.setPower(-0.2); // set to negative of the joystick input not a random decimal
//            }
//
//            if (lifterPos < 10) {
//                lifterMotor.setPower(0.1);
//            }

            armMotor.setPower(operatorArm * 0.75); // multiply by some decimal less than 1 to slow the arm acceleration
            armMotor2.setPower(operatorArm * 0.75);

            if (gamepad2.b) {
                lifterMotor.setTargetPosition(lifterUP);
                lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lifterMotor.setPower(1);
            }
            if (gamepad2.a) {
                lifterMotor.setTargetPosition(lifterHang);
                lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lifterMotor.setPower(1);
            }
            if (gamepad2.back) {
                lifterMotor.setTargetPosition(lifterDown);
                lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lifterMotor.setPower(1);
            }
//            if (gamepad2.y) {
//                armMotor.setTargetPosition(armUP);
//                armMotor2.setTargetPosition(armUP);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(0.5);
//                armMotor2.setPower(0.5);
//            }
//            if (gamepad2.x) {
//                armMotor.setTargetPosition(armDown);
//                armMotor2.setTargetPosition(armDown);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(0.5);
//                armMotor2.setPower(0.5);
//            }

//            lifterMotor.setPower(lifterThing);

//            telemetry.addData("armPos", armPos);
            telemetry.addData("Heading", botHeading);
            telemetry.addData("lifterPos", lifterPos);

//            frontLeftMotor.setPower(frontLeftPower); // drive motor things
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

            telemetry.update();

        }
    }
}
