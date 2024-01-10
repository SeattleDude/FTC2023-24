package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public class AutoBlueStageShort extends LinearOpMode {

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

//    String TFOD_MODEL_ASSET = "BLUEmodel_20231211_180224.tflite";
    String TFOD_MODEL_ASSET = "REDmodel_20240106_120831.tflite";
    private static final String[] LABELS = {
            "Viking"
    };

    SpikeDetectionPos pos = SpikeDetectionPos.UNK;
    SampleMecanumDrive drive;

    Trajectory centerDropOne;

    Trajectory centerDropTwo;

    Trajectory leftDrop;
    Trajectory leftDropOne;
    Trajectory leftDropTwo;

    Trajectory rightDropOne;
    Trajectory rightDropTwo;
    Trajectory leftDetect;
    Trajectory sideDropTwo;

    Servo purplePixelDropper;

//    Servo yellowPixelDropper = hardwareMap.servo.get("yellowPixel");

    double purplePixelServoUp = 0.8;
    double purplePixelServoDown = 0;

    TrajectorySequence waitOneSec;

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
        initTfod();

        while (opModeIsActive()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);

        }

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory firstTrajectory = drive.trajectoryBuilder(startPose,true)
                .back(16)
                .build();
//        leftDrop = drive.trajectoryBuilder(firstTrajectory.end(), true)
//                .splineTo(new Vector2d(14,34), 0)
//                .build();



        leftDetect = drive.trajectoryBuilder(firstTrajectory.end())
                .strafeTo(new Vector2d(24, 43))
                .build();


        centerDropOne = drive.trajectoryBuilder(firstTrajectory.end(),true)
                        .back(23)
                                .build();
        centerDropTwo = drive.trajectoryBuilder(centerDropOne.end(),true)
                        .forward(13)
                                .build();

        leftDropOne = drive.trajectoryBuilder(firstTrajectory.end(),true)
                .back(3)
                .build();

//        leftDropTwo = drive.trajectoryBuilder(leftDropOne.end(),true)
//                .back(2)
//                .build();

        rightDropOne = drive.trajectoryBuilder(new Pose2d(firstTrajectory.end().getX(),firstTrajectory.end().getY(),315),0)
                .back(8)
                .build();

//        rightDropTwo = drive.trajectoryBuilder(rightDropOne.end(),true)
//                .back(5)
//                .build();

        Trajectory secondTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
                .splineTo(new Vector2d(46, 58), 0, SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory thirdTrajectory = drive.trajectoryBuilder(secondTrajectory.end())
                .back(2)
                .build();

        waitOneSec = drive.trajectorySequenceBuilder(firstTrajectory.end())
                        .waitSeconds(2)
                        .build();

        waitForStart();

        if(isStopRequested()) return;

//        armMotor.setTargetPosition(-55);
//        armMotor2.setTargetPosition(-55);
//        armMotor.setPower(-0.5);
//        armMotor2.setPower(-0.5);
//        drive.followTrajectory(firstTrajectory);
//        drive.followTrajectory(secondTrajectory);
//        armMotor.setTargetPosition(0);
//        armMotor2.setTargetPosition(0);
//        armMotor.setPower(0.5);
//        armMotor2.setPower(0.5);
//        Thread.sleep(1000);
//        drive.followTrajectory(thirdTrajectory);

        drive.followTrajectory(firstTrajectory);
        drive.followTrajectorySequence(waitOneSec);
        spikeDetector();
        drive.followTrajectorySequence(waitOneSec);
        purplePixelDropper.setPosition(purplePixelServoUp);
        drive.followTrajectorySequence(waitOneSec);
//        drive.followTrajectory(secondTrajectory);
//        drive.followTrajectory(thirdTrajectory);

    }

    void spikeDetector() {
        //if there is one object detected before turning then the object is the center
        //yaaay works
        for(int i = 0; i<10; i++)
        {
            telemetryTfod();
        }
        if (currentRecognitions != null && currentRecognitions.size()==1) {
           drive.followTrajectory(centerDropOne);
           drive.followTrajectory(centerDropTwo);
           purplePixelDropper.setPosition(purplePixelServoDown);
           pos = SpikeDetectionPos.CENTER;
           return;
        }
//        drive.followTrajectory(leftDetect);
       //turn 45 degrees left
       drive.turn(Math.toRadians(45));
        drive.followTrajectorySequence(waitOneSec);
        for(int j = 0; j<10; j++)
        {
            telemetryTfod();
        }
        //if there is one object detected after turning 45 degrees left the object is in the left
        //yaaay works
        if(currentRecognitions != null && currentRecognitions.size()==1){
            drive.turn(Math.toRadians(-45));
            drive.followTrajectory(leftDropOne);
            drive.turn(Math.toRadians(90));
//            drive.followTrajectory(leftDropTwo);
            purplePixelDropper.setPosition(purplePixelServoDown);
            pos = SpikeDetectionPos.LEFT;
            return;
       }
       //if its not center or left it is on right
        //doesnt works:(
        drive.turn(Math.toRadians(-90));
       if (true) {
           drive.followTrajectorySequence(waitOneSec);
           drive.followTrajectory(rightDropOne); //broked
//            drive.followTrajectory(rightDropTwo);
           purplePixelDropper.setPosition(purplePixelServoDown);
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
    double x;
    double y;

    List<Recognition> currentRecognitions;
    private void telemetryTfod() {

        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}
