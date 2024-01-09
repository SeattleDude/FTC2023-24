package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Team 23974 A.S.T.R.O. Vikings, Centerstage 2023-2024
@Autonomous
public class AutoRedStageLong extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor armMotor = hardwareMap.dcMotor.get("armer");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("armer2");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // gotta restart the motor after
                                                                // stopping to zero the encoder
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);

        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -72, Math.toRadians(90)); // could make this a variable
        // in a different file?
//        Pose2d secondPose = new Pose2d(12, 48, Math.toRadians(270));   // shouldn't need this?

        drive.setPoseEstimate(startPose);

        Trajectory firstTrajectory = drive.trajectoryBuilder(startPose)
                .forward(5)
                .build();

        Trajectory secondTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
                .splineTo(new Vector2d(65, -50), 0)
                .build();

        Trajectory thirdTrajectory = drive.trajectoryBuilder(secondTrajectory.end())
                .back(2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        armMotor.setTargetPosition(-60);
        armMotor2.setTargetPosition(-60);
        armMotor.setPower(-0.5);
        armMotor2.setPower(-0.5);
        drive.followTrajectory(firstTrajectory);
        drive.followTrajectory(secondTrajectory);
        armMotor.setTargetPosition(0);
        armMotor2.setTargetPosition(0);
        armMotor.setPower(0.5);
        armMotor2.setPower(0.5);
        Thread.sleep(1000);
        drive.followTrajectory(thirdTrajectory);

    }

}
