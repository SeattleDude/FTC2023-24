package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class AutoBlueStage extends LinearOpMode {
    SpikeDetectionPos Pos = SpikeDetectionPos.UNK;

//    private static final boolean USE_WEBCAM = true;
//    private TfodProcessor tfod;
//    private VisionPortal visionPortal;
//    List<Recognition> RecognitionList = tfod.getRecognitions();


    @Override
    public void runOpMode() {

//        initTfod();

        while (opModeIsActive()) {

//            telemetryTfod();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
//                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
//                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);

        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 72, Math.toRadians(270));
        Pose2d secondPose = new Pose2d(12, 48, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();

        Trajectory secondTrajectory = drive.trajectoryBuilder(secondPose)
                .splineTo(new Vector2d(72-26, 30), 0)
                .build();

        waitForStart();

        if(isStopRequested()) return;

//        drive.followTrajectory(myTrajectory);
        // do vision processing here
//        drive.followTrajectory(secondTrajectory);

//        visionPortal.close();

    }

//    private void initTfod() {
//
//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // Use setModelAssetName() if the TF Model is built in as an asset.
//                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                //.setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                //.setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                //.setModelAspectRatio(16.0 / 9.0)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableCameraMonitoring(true);
//        builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        //tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }
//
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        if (currentRecognitions.size() == 1) {
//
//            for (int i=0; i < 5; i++) {
////                servo.setPosition(0);
////                servo.setPosition(0.5);
//            }
//        }
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
//    }
//
//    public void getXY() {
//        double x = 0;
//        double y = 0;
//        for (Recognition recognition : RecognitionList) {
//            x = (recognition.getLeft() + recognition.getRight()) / 2;
//            y = (recognition.getTop() + recognition.getBottom()) / 2;
//        }
//
//    }

}
