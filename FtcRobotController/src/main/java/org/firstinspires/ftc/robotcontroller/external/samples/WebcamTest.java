package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Webcam with AprilTag & Color Detection", group = "Samples")
public class WebcamTest extends LinearOpMode {
    OpenCvCamera webcam;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Setup webcam for color detection
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize AprilTag Processor
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Setup VisionPortal for AprilTag detection
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(CameraName.class, "Webcam 1"), myAprilTagProcessor);

        // Set pipeline for color detection
        ColorDetectionPipeline pipeline = new ColorDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480);
                telemetry.addData("Status", "Webcam Opened");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Webcam could not start");
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Main loop for AprilTag and color detection
        while (opModeIsActive()) {
            // AprilTag detection
            int tagCount = myAprilTagProcessor.getDetections().size();
            telemetry.addData("AprilTags Detected", tagCount);

            if (tagCount > 0) {
                telemetry.addData("First Tag ID", myAprilTagProcessor.getDetections().get(0).id);
            }

            // Color detection
            telemetry.addData("Detected Color", pipeline.getDetectedColor());

            telemetry.update();
        }

        // Stop camera streaming
        webcam.stopStreaming();
        visionPortal.close();
    }

    /**
     * Color Detection Pipeline: Processes camera input and detects Red, Blue, and Yellow.
     */
    static class ColorDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "None";

        public String getDetectedColor() {
            return detectedColor;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define HSV ranges for each color
            Scalar lowerRed1 = new Scalar(0, 120, 70);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 120, 70);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            // Create masks for colors
            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, redMask1);
            Core.inRange(hsv, lowerRed2, upperRed2, redMask2);

            Mat redMask = new Mat();
            Core.addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask);

            Mat blueMask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, blueMask);

            Mat yellowMask = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

            // Count the non-zero pixels
            int redCount = Core.countNonZero(redMask);
            int blueCount = Core.countNonZero(blueMask);
            int yellowCount = Core.countNonZero(yellowMask);

            // Detect the color based on thresholds
            if (redCount > 5000) {
                detectedColor = "Red";
            } else if (blueCount > 5000) {
                detectedColor = "Blue";
            } else if (yellowCount > 5000) {
                detectedColor = "Yellow";
            } else {
                detectedColor = "None";
            }

            // Return the original input
            return input;
        }
    }
}