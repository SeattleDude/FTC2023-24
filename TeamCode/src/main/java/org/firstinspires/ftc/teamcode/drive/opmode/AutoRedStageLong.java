package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    Vector2d detectPos = new Vector2d(12, 0);

    Vector2d dropPos = new Vector2d(25, 0);
    Vector2d leftSpikeMark = new Vector2d(25, 0); // SET THIS CORRECTLY
    Vector2d rightSpikeMark = new Vector2d(25, 0);

    @Override
    public void runOpMode() throws InterruptedException{
        purplePixelDropper = hardwareMap.servo.get("purplePixel");
        DcMotor armMotor = hardwareMap.dcMotor.get("armer");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("armer2");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // gotta restart the motor after stopping to zero the encoder
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double slowerVelocity = 25;
        TrajectoryVelocityConstraint velCons = SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        initTfod();

        while (opModeIsActive()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);

        }

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();
        if(isStopRequested()) return;

        Trajectory centerDrop = drive.trajectoryBuilder(new Pose2d(detectPos.getX(), detectPos.getY(), 0))
                .strafeTo(dropPos)
                .addDisplacementMarker(() -> {
                    purplePixelDropper.setPosition(purplePixelServoDown);
                })
                .build();

        TrajectorySequence leftDrop = drive.trajectorySequenceBuilder(new Pose2d(detectPos.getX(), detectPos.getY(), 0))
                .strafeTo(dropPos)
                .turn(Math.toRadians(90)) // 90 deg left
                .addDisplacementMarker(() -> {
                    purplePixelDropper.setPosition(purplePixelServoDown);
                })
                .build();

        TrajectorySequence rightDrop = drive.trajectorySequenceBuilder(new Pose2d(detectPos.getX(), detectPos.getY(), 0))
                .strafeTo(dropPos)
                .turn(Math.toRadians(-90)) // 90 deg left
                .addDisplacementMarker(() -> {
                    purplePixelDropper.setPosition(purplePixelServoDown);
                })
                .build();


        TrajectorySequence FunkyForward = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(velCons)
                .strafeTo(detectPos)
                .addDisplacementMarker(() -> {
                    SpikeDetect(10);
                })
                .build();

        drive.followTrajectorySequence(FunkyForward);
        switch (pos) {
            case CENTER:
                drive.followTrajectory(centerDrop);
                break;
            case LEFT:
                drive.followTrajectorySequence(leftDrop);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightDrop);
                break;
            case UNK:
                telemetry.addData("brok me", "");
                break;
        }


    }

    public void SpikeDetect(int count)
    {

        // pointing at CENTER spike mark from detectPos
        for (int i = 0; i < count; i++) {
            telemetryTfod();
        }
        if (currentRecognitions != null && currentRecognitions.size()==1) { // detect center
            pos = SpikeDetectionPos.CENTER;
            return;
        }
        drive.turn(35); // turn left to setup for detecting left
        for (int i = 0; i < count; i++) {
            telemetryTfod();
        }
        if (currentRecognitions != null && currentRecognitions.size()==1) { // detect left
            pos = SpikeDetectionPos.LEFT;
            drive.turn(-35); // turn back to face straight
        }
        else {
            drive.turn(-35); // turn back to face straight
            pos = SpikeDetectionPos.RIGHT;
        }

    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
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

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true); // LOOK AT THIS, this looks like a
        // good way to start and stop the vision processing when we go to detect the pixel on the
        // spike mark

    }   // end method initTfod()

    List<Recognition> currentRecognitions;
    private void telemetryTfod() {

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
        }   // end for() loop

    }   // end method telemetryTfod()
}
