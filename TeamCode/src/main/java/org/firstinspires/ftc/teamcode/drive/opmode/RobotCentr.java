package org.firstinspires.ftc.teamcode.drive.opmode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class RobotCentr extends LinearOpMode{

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    Double yellowPixelUp = 0.1;
    Double yellowPixelDown = 0.45;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");
        DcMotor armMotor = hardwareMap.dcMotor.get("armer");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("armer2");
        Servo planeServo = hardwareMap.servo.get("droneLauncher");
        Servo purplePixelServo = hardwareMap.get(Servo.class,"purplePixel");
        Servo yellowPixelServo = hardwareMap.get(Servo.class,"yellowPixel");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double pose = drive.getExternalHeading(); // used for my custom function to drive straight thru the trusses

            double slowMode = gamepad1.left_trigger;
            double slowCoeff = 0.3;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            boolean aimer = gamepad1.a; // set so that we can make an auto straight driver method
            // so that we can easily make our way thru the trusses

            double lifterThing = -gamepad2.right_stick_y;
            double operatorArm = -gamepad2.left_stick_y;
            boolean plane = false;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            if (aimer) {

                yellowPixelServo.setPosition(yellowPixelDown);

            } else {
                yellowPixelServo.setPosition(yellowPixelUp);
            }

            // Send calculated power to wheels
            if (slowMode > 0.2) {

                leftFrontDrive.setPower(leftFrontPower * slowCoeff);
                rightFrontDrive.setPower(rightFrontPower * slowCoeff);
                leftBackDrive.setPower(leftBackPower * slowCoeff);
                rightBackDrive.setPower(rightBackPower * slowCoeff);

            } else {

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

            }

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setPower(operatorArm * 0.4);
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor2.setPower(operatorArm * 0.4);
            lifterMotor.setPower(lifterThing);
            int armPos = armMotor.getCurrentPosition();

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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

}
