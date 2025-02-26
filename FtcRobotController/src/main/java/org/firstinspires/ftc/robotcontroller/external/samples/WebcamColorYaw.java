package org.firstinspires.ftc.robotcontroller.external.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Webcam Color Yaw", group = "Main")
public class WebcamColorYaw extends LinearOpMode {
    private OpenCvCamera webcam;
    private IMU imu;
    private Servo intake;
    private Servo intakelr;
    private ColorDetectionPipeline colorPipeline;
    private static final double CAMERA_FOV_DEGREES = 60.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        intake = hardwareMap.get(Servo.class, "intake");
        intakelr = hardwareMap.get(Servo.class, "intakelr");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        colorPipeline = new ColorDetectionPipeline();
        webcam.setPipeline(colorPipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double robotYaw = orientation.getYaw(AngleUnit.DEGREES);

            String detectedColor = colorPipeline.getDetectedColor();
            double sampleYawOffset = colorPipeline.getSampleYawOffset();
            double sampleYaw = robotYaw + sampleYawOffset;
            String orientationType = colorPipeline.getOrientationType();

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Sample Yaw Offset", "%.2f degrees", sampleYawOffset);
            telemetry.addData("Sample Yaw", "%.2f degrees", sampleYaw);
            telemetry.addData("Object Orientation", orientationType);
            telemetry.update();

            if (Math.abs(sampleYawOffset) < 5) { // Sample is centered
                if ("Vertical".equals(orientationType)) {
                    intake.setPosition(1.0); // Close intake
                } else if ("Horizontal".equals(orientationType)) {
                    intake.setPosition(0);
                    intakelr.setPosition(1.0); // Rotate until vertical
                    sleep(500); // Allow time for rotation
                    intakelr.setPosition(0.5); // Stop rotation
                    intake.setPosition(1.0); // Close intake
                } else {
                    intake.setPosition(0);
                }
            }
        }
    }

    static class ColorDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "None";
        private double sampleYawOffset = 0.0;
        private static final int FRAME_WIDTH = 640;
        private static final int FRAME_HEIGHT = 480;
        private String orientationType = "Unknown";

        // Known sample dimensions (in inches)
        private static final double SAMPLE_WIDTH_INCHES = 3.5;
        private static final double SAMPLE_HEIGHT_INCHES = 1.5;

        // Known aspect ratio of the sample (width / height)
        private static final double SAMPLE_ASPECT_RATIO_HORIZONTAL = SAMPLE_WIDTH_INCHES / SAMPLE_HEIGHT_INCHES; // ≈ 2.33
        private static final double SAMPLE_ASPECT_RATIO_VERTICAL = SAMPLE_HEIGHT_INCHES / SAMPLE_WIDTH_INCHES;   // ≈ 0.43
        private static final double ASPECT_RATIO_TOLERANCE = 0.3; // Allow some tolerance for detection

        // Expected sample size in pixels (calculated based on FOV and resolution)
        private static final double PIXELS_PER_INCH = FRAME_WIDTH / (2 * Math.tan(Math.toRadians(CAMERA_FOV_DEGREES / 2)) * SAMPLE_WIDTH_INCHES);
        private static final double EXPECTED_SAMPLE_WIDTH_PX = SAMPLE_WIDTH_INCHES * PIXELS_PER_INCH;
        private static final double EXPECTED_SAMPLE_HEIGHT_PX = SAMPLE_HEIGHT_INCHES * PIXELS_PER_INCH;
        private static final double SIZE_TOLERANCE = 0.5; // Allow 50% tolerance for size detection (adjust as needed)

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Define color ranges for detection
            Scalar lowerRed = new Scalar(0, 120, 70);
            Scalar upperRed = new Scalar(10, 255, 255);
            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            // Create masks for each color
            Mat redMask = new Mat();
            Mat blueMask = new Mat();
            Mat yellowMask = new Mat();

            Core.inRange(hsvMat, lowerRed, upperRed, redMask);
            Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);
            Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);

            // Combine masks to detect any of the colors
            Mat combinedMask = new Mat();
            Core.bitwise_or(redMask, blueMask, combinedMask);
            Core.bitwise_or(combinedMask, yellowMask, combinedMask);

            // Preprocess the mask to reduce noise
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours in the combined mask
            Mat contoursMat = new Mat();
            combinedMask.copyTo(contoursMat);
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(contoursMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest contour (assume it's the sample)
            Rect boundingRect = new Rect();
            if (!contours.isEmpty()) {
                double maxArea = 0;
                for (MatOfPoint contour : contours) {
                    Rect rect = Imgproc.boundingRect(contour);
                    double area = rect.area();

                    // Check if the object matches the expected size and aspect ratio
                    double aspectRatio = (double) rect.width / rect.height;
                    boolean isCorrectSize = Math.abs(rect.width - EXPECTED_SAMPLE_WIDTH_PX) < EXPECTED_SAMPLE_WIDTH_PX * SIZE_TOLERANCE &&
                            Math.abs(rect.height - EXPECTED_SAMPLE_HEIGHT_PX) < EXPECTED_SAMPLE_HEIGHT_PX * SIZE_TOLERANCE;
                    boolean isCorrectAspectRatio = Math.abs(aspectRatio - SAMPLE_ASPECT_RATIO_HORIZONTAL) < ASPECT_RATIO_TOLERANCE ||
                            Math.abs(aspectRatio - SAMPLE_ASPECT_RATIO_VERTICAL) < ASPECT_RATIO_TOLERANCE;

                    if (area > maxArea && isCorrectSize && isCorrectAspectRatio) {
                        maxArea = area;
                        boundingRect = rect;
                    }
                }

                if (maxArea > 0) { // Valid sample detected
                    // Draw a green border around the sample
                    Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);

                    // Calculate the aspect ratio of the bounding rectangle
                    double aspectRatio = (double) boundingRect.width / boundingRect.height;

                    // Determine orientation based on the known aspect ratio
                    if (Math.abs(aspectRatio - SAMPLE_ASPECT_RATIO_HORIZONTAL) < ASPECT_RATIO_TOLERANCE) {
                        orientationType = "Horizontal";
                    } else if (Math.abs(aspectRatio - SAMPLE_ASPECT_RATIO_VERTICAL) < ASPECT_RATIO_TOLERANCE) {
                        orientationType = "Vertical";
                    } else {
                        orientationType = "Unknown";
                    }

                    // Calculate the yaw offset
                    double centerX = boundingRect.x + (boundingRect.width / 2.0);
                    double normalizedX = (centerX - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
                    sampleYawOffset = normalizedX * (CAMERA_FOV_DEGREES / 2.0);

                    // Determine the detected color
                    double redPixels = Core.countNonZero(redMask);
                    double bluePixels = Core.countNonZero(blueMask);
                    double yellowPixels = Core.countNonZero(yellowMask);

                    if (redPixels > bluePixels && redPixels > yellowPixels) {
                        detectedColor = "Red";
                    } else if (bluePixels > redPixels && bluePixels > yellowPixels) {
                        detectedColor = "Blue";
                    } else if (yellowPixels > redPixels && yellowPixels > bluePixels) {
                        detectedColor = "Yellow";
                    } else {
                        detectedColor = "None";
                    }

                    // Display the detected color and orientation on the frame
                    Imgproc.putText(input, detectedColor + " - " + orientationType,
                            new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
                }
            }

            // Release memory
            redMask.release();
            blueMask.release();
            yellowMask.release();
            combinedMask.release();
            contoursMat.release();
            hsvMat.release();
            return input;
        }

        public String getDetectedColor() {
            return detectedColor;
        }

        public double getSampleYawOffset() {
            return sampleYawOffset;
        }

        public String getOrientationType() {
            return orientationType;
        }
    }
}