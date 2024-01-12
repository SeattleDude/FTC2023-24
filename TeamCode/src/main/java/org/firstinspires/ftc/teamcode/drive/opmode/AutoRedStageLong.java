package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


// Team 23974 A.S.T.R.O. Vikings, Centerstage 2023-2024
@Autonomous
public class AutoRedStageLong extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

//    public Servo servo;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    String TFOD_MODEL_ASSET = "BLUEmodel_20231211_180224.tflite";
//    String TFOD_MODEL_ASSET = "REDmodel_20240106_120831.tflite";
    private static final String[] LABELS = {
            "Viking"
    };

    SpikeDetectionPos pos = SpikeDetectionPos.UNK;
    SampleMecanumDrive drive;

    Servo purplePixelDropper;

//    Servo yellowPixelDropper = hardwareMap.servo.get("yellowPixel");

    double purplePixelServoUp = 0.8;
    double purplePixelServoDown = 0;


    TrajectorySequence centerDrop;
    TrajectorySequence centerDrop2;
    TrajectorySequence leftDrop;
    TrajectorySequence leftDrop2;
    TrajectorySequence leftDrop3;

    TrajectorySequence rightDrop;
    TrajectorySequence rightDrop2;

    TrajectorySequence rightDrop3;

    TrajectorySequence waitSome;


    Vector2d detectPos = new Vector2d(12, 0);
    Vector2d dropPos = new Vector2d(25, 0);

    @Override
    public void runOpMode() {
        purplePixelDropper = hardwareMap.servo.get("purplePixel");
        double slowerVelocity = 25; // 25 is half ish of the full speed
        TrajectoryVelocityConstraint velCons = SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accCons = SampleMecanumDrive.getAccelerationConstraint(slowerVelocity);
        initTfod();

        while (opModeIsActive()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);

        }

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory FunkyForward = drive.trajectoryBuilder(startPose)
//                .setVelConstraint(velCons) // slow the trajectorySequence down
//                .strafeTo(detectPos) // go in a straight line on the Vector2d
                .lineTo(detectPos, velCons, accCons) // make it simple????? IDK anymore dude - "M" at 1/10/24 8:05PM
                .build(); // build it

        centerDrop = drive.trajectorySequenceBuilder(new Pose2d(detectPos.getX(), detectPos.getY(), Math.toRadians(180)))
                .lineTo(new Vector2d(32, 0), velCons, accCons)
                .build();

        centerDrop2 = drive.trajectorySequenceBuilder(centerDrop.end())
                .lineTo(new Vector2d(25, 0), velCons, accCons)
                .build();

        leftDrop = drive.trajectorySequenceBuilder(new Pose2d(detectPos.getX(), detectPos.getY(), Math.toRadians(180)))
                .lineTo(new Vector2d(dropPos.getX() + 3, dropPos.getY()), velCons, accCons)
                .turn(Math.toRadians(90))
                .build();

        leftDrop2 = drive.trajectorySequenceBuilder(leftDrop.end())
                .back(5)
                .build();

        leftDrop3 = drive.trajectorySequenceBuilder(leftDrop2.end())
                .forward(6)
                .build();

        rightDrop = drive.trajectorySequenceBuilder(new Pose2d(detectPos.getX(), detectPos.getY(), Math.toRadians(180)))
                .lineTo(new Vector2d(dropPos.getX() + 3, dropPos.getY()), velCons, accCons)
                .turn(Math.toRadians(-90))
                .build();

        rightDrop2 = drive.trajectorySequenceBuilder(rightDrop.end())
                .back(5)
                .build();

        rightDrop3 = drive.trajectorySequenceBuilder(rightDrop2.end())
                .forward(6)
                .build();

        waitSome = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .build();

        waitForStart(); /*****  DON'T RUN ANY MOTOR MOVEMENT ABOVE THIS LINE!! You WILL get PENALTIES! And it's UNSAFE! *****/
        if (isStopRequested()) return;

        /***** start of manual code running or initiation or whatever *****/

        drive.followTrajectory(FunkyForward); // go forward to the distance we need to detect at
        label:
        {
            drive.followTrajectorySequence(waitSome);
            for (int i=0; i<10; i++) {
                telemetryTfod(); // detect
            }
            if (currentRecognitions != null && currentRecognitions.size() == 1) {
                pos = SpikeDetectionPos.CENTER;
                break label;
            }
            drive.turn(Math.toRadians(35)); // left 35
            drive.followTrajectorySequence(waitSome);
            for (int i=0; i<10; i++) {
                telemetryTfod(); // detect
            }
            if (currentRecognitions != null && currentRecognitions.size() == 1) {
                pos = SpikeDetectionPos.LEFT;
                drive.turn(Math.toRadians(-35)); // face back forward
                break label;
            }
            pos = SpikeDetectionPos.RIGHT;
            drive.turn(Math.toRadians(-35)); // turn to face forward
        }
        if (pos == SpikeDetectionPos.CENTER) {
            drive.followTrajectorySequence(centerDrop);
            drive.followTrajectorySequence(centerDrop2);
            purplePixelDropper.setPosition(purplePixelServoDown);
            drive.followTrajectorySequence(waitSome);
            purplePixelDropper.setPosition(purplePixelServoUp);
            drive.followTrajectorySequence(waitSome);
        }
        else if (pos == SpikeDetectionPos.LEFT) {
            drive.followTrajectorySequence(leftDrop);
            drive.followTrajectorySequence(leftDrop2);
            drive.followTrajectorySequence(leftDrop3);
            purplePixelDropper.setPosition(purplePixelServoDown);
            drive.followTrajectorySequence(waitSome);
            purplePixelDropper.setPosition(purplePixelServoUp);
            drive.followTrajectorySequence(waitSome);
        }
        else if (pos == SpikeDetectionPos.RIGHT) {
            drive.followTrajectorySequence(rightDrop);
            drive.followTrajectorySequence(rightDrop2);
            drive.followTrajectorySequence(rightDrop3);
            purplePixelDropper.setPosition(purplePixelServoDown);
            drive.followTrajectorySequence(waitSome);
            purplePixelDropper.setPosition(purplePixelServoUp);
            drive.followTrajectorySequence(waitSome);
        }


        // detect straight, if on straight drive to dropPos and drop pixel
        // turn left ~35 ish degrees
        // detect left , if on left drive to dropPos, turn left 90, drive forward then back 5 inches, and drop pixel
        // if not on left, turn right ~35 ish degrees
        // drive to dropPos, turn right 90, drive forward then back 5 inches, and drop pixel


//        purplePixelDropper.setPosition(purplePixelServoUp); // reset the position of the pixel dropper to up for hardware safety

        /***** end of manual code running or initiation or whatever *****/
    }


    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setModelAspectRatio(4.0 / 3.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions. // should probably set this correctly
        builder.setCameraResolution(new Size(1280, 720));
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);
        builder.enableLiveView(true);
        // Set and enable the processor.
        builder.addProcessor(tfod);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.85f);
        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true); // LOOK AT THIS, this looks like a
        // good way to start and stop the vision processing when we go to detect the pixel on the
        // spike mark
    }   // end method initTfod()

    List<Recognition> currentRecognitions;
    private void telemetryTfod() {

        telemetry.clearAll();
        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData(""," ");
            telemetry.addData("Detected pos: ",pos); // this should allow us to see where the robot thinks the prop is
        }   // end for() loop

    }   // end method telemetryTfod()
}
