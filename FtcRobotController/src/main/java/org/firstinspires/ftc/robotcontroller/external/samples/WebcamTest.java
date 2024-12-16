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

@Autonomous(name = "Webcam Color Detection", group = "Samples")
public class WebcamTest extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // Get the camera monitor view ID for the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set pipeline for processing
        ColorDetectionPipeline pipeline = new ColorDetectionPipeline();
        webcam.setPipeline(pipeline);

        // Open the webcam asynchronously
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

        // Main loop to display detected colors
        while (opModeIsActive()) {
            telemetry.addData("Detected Color", pipeline.getDetectedColor());
            telemetry.update();
        }

        // Stop the camera streaming
        webcam.stopStreaming();
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
            if (redCount > 1000) {
                detectedColor = "Red";
            } else if (blueCount > 1000) {
                detectedColor = "Blue";
            } else if (yellowCount > 1000) {
                detectedColor = "Yellow";
            } else {
                detectedColor = "None";
            }

            // Return the original input
            return input;
        }
    }
}