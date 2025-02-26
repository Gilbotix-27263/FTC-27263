package org.firstinspires.ftc.robotcontroller.external.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "Webcam Color Yaw", group = "Main")
public class WebcamColorYaw extends LinearOpMode {
    private OpenCvCamera webcam;
    private IMU imu;
    private Servo intake;
    private Servo intakelr;
    private ShapeDetectionPipeline colorPipeline;
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
        colorPipeline = new ShapeDetectionPipeline();
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

        // Initialize IMU with parameters
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Set initial servo positions
        intake.setPosition(0); // Open intake
        intakelr.setPosition(0.5); // Neutral position for rotation

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Reset IMU heading when the match starts
        imu.resetYaw();

        while (opModeIsActive()) {
            // Get robot orientation from IMU
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double robotYaw = orientation.getYaw(AngleUnit.DEGREES);

            // Get data from the vision pipeline
            String detectedElement = colorPipeline.getDetectedElement();
            double elementYawOffset = colorPipeline.getElementYawOffset();
            String orientationType = colorPipeline.getOrientationType();
            double confidence = colorPipeline.getConfidence();

            // Calculate absolute yaw of the sample in field coordinates
            double sampleYaw = robotYaw + elementYawOffset;

            // Display telemetry data
            telemetry.addData("Detected Element", detectedElement);
            telemetry.addData("Element Yaw Offset", "%.2f degrees", elementYawOffset);
            telemetry.addData("Robot Yaw", "%.2f degrees", robotYaw);
            telemetry.addData("Sample Yaw (Field)", "%.2f degrees", sampleYaw);
            telemetry.addData("Object Orientation", orientationType);
            telemetry.addData("Detection Confidence", "%.1f%%", confidence);

            // Only attempt to grab if a game element is detected
            if ("GameElement".equals(detectedElement) && Math.abs(elementYawOffset) < 5) { // Element is centered
                telemetry.addData("Status", "Element centered - attempting to grab");

                if ("Vertical".equals(orientationType)) {
                    // Element already vertical, just close intake
                    intake.setPosition(1.0); // Close intake
                    telemetry.addData("Action", "Grabbing vertical element");
                } else if ("Horizontal".equals(orientationType)) {
                    // Element is horizontal, need to rotate first
                    intake.setPosition(0); // Open intake
                    intakelr.setPosition(1.0); // Rotate until vertical
                    telemetry.addData("Action", "Rotating horizontal element");
                    sleep(500); // Allow time for rotation
                    intakelr.setPosition(0.5); // Stop rotation
                    intake.setPosition(1.0); // Close intake
                    telemetry.addData("Action", "Grabbing after rotation");
                } else {
                    // Unknown orientation, keep intake open
                    intake.setPosition(0);
                    telemetry.addData("Action", "Unknown orientation - intake open");
                }
            } else if ("GameElement".equals(detectedElement)) {
                // Element detected but not centered
                telemetry.addData("Status", "Element detected but not centered");
                intake.setPosition(0); // Keep intake open

                // Optional: add auto-centering behavior here
            } else {
                // No element detected
                telemetry.addData("Status", "No game element detected");
                intake.setPosition(0); // Keep intake open
            }

            // Manual override controls
            if (gamepad1.a) {
                intake.setPosition(0); // Open intake
                telemetry.addData("Manual", "Intake opened");
            } else if (gamepad1.b) {
                intake.setPosition(1.0); // Close intake
                telemetry.addData("Manual", "Intake closed");
            }

            if (gamepad1.left_bumper) {
                intakelr.setPosition(0.0); // Rotate one direction
                telemetry.addData("Manual", "Rotating left");
            } else if (gamepad1.right_bumper) {
                intakelr.setPosition(1.0); // Rotate other direction
                telemetry.addData("Manual", "Rotating right");
            } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                intakelr.setPosition(0.5); // Stop rotation
            }

            telemetry.update();
        }

        // Clean up
        webcam.stopStreaming();
        FtcDashboard.getInstance().stopCameraStream();
    }
}

    class ShapeDetectionPipeline extends OpenCvPipeline {
        private String detectedElement = "None";
        private double elementYawOffset = 0.0;
        private static final int FRAME_WIDTH = 640;
        private static final int FRAME_HEIGHT = 480;
        private String orientationType = "Unknown";
        private double confidence = 0.0;

        // Visualization options
        private boolean showDebugOverlay = true;
        private Point lastDetectedCenter = null;
        private Rect lastDetectedRect = null;

        // Detection highlight colors
        private static final Scalar HIGHLIGHT_COLOR = new Scalar(0, 255, 0); // Green
        private static final Scalar TARGET_LINE_COLOR = new Scalar(255, 0, 0); // Red
        private static final Scalar TEXT_COLOR = new Scalar(255, 255, 255); // White

        // Camera parameters
        private static final double CAMERA_FOV_DEGREES = 60.0; // Update this to match your camera's FOV

        // Game element dimensions (in inches) - adjust to match your game elements
        private static final double ELEMENT_WIDTH_INCHES = 3.5;
        private static final double ELEMENT_HEIGHT_INCHES = 1.5;

        // Expected aspect ratios for different orientations
        private static final double ASPECT_RATIO_HORIZONTAL = ELEMENT_WIDTH_INCHES / ELEMENT_HEIGHT_INCHES;
        private static final double ASPECT_RATIO_VERTICAL = ELEMENT_HEIGHT_INCHES / ELEMENT_WIDTH_INCHES;
        private static final double ASPECT_RATIO_TOLERANCE = 0.5;

        // Tracking history for stable detection
        private static final int DETECTION_HISTORY_SIZE = 5;
        private final LinkedList<String> elementHistory = new LinkedList<>();
        private final LinkedList<String> orientationHistory = new LinkedList<>();
        private final LinkedList<Rect> rectHistory = new LinkedList<>();

        // Detection parameters
        private static final double MIN_CONTOUR_AREA = 500;
        private static final double MAX_CONTOUR_AREA = FRAME_WIDTH * FRAME_HEIGHT / 4; // Max 1/4 of frame

        // Frame counters for FPS calculation
        private int frameCount = 0;
        private long lastFpsTimestamp = System.currentTimeMillis();
        private double fps = 0;

        // Animation parameters for highlight effect
        private int highlightThickness = 2;
        private int animationDirection = 1;
        private static final int MIN_HIGHLIGHT_THICKNESS = 2;
        private static final int MAX_HIGHLIGHT_THICKNESS = 5;

        @Override
        public Mat processFrame(Mat input) {
            // Create working matrices
            Mat grayMat = new Mat();
            Mat blurredMat = new Mat();
            Mat thresholdMat = new Mat();
            Mat cannyOutput = new Mat();
            Mat hierarchy = new Mat();

            // Convert to grayscale for better shape detection
            Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_RGB2GRAY);

            // Apply blur to reduce noise
            Imgproc.GaussianBlur(grayMat, blurredMat, new Size(5, 5), 0);

            // Apply adaptive threshold to handle varying lighting conditions
            Imgproc.adaptiveThreshold(blurredMat, thresholdMat, 255,
                    Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 11, 2);

            // Apply morphological operations to clean up the image
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_OPEN, kernel);

            // Find edges
            Imgproc.Canny(thresholdMat, cannyOutput, 50, 150);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(cannyOutput, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Sort contours by area (largest first)
            contours.sort((c1, c2) -> Double.compare(Imgproc.contourArea(c2), Imgproc.contourArea(c1)));

            String currentDetectedElement = "None";
            String currentOrientation = "Unknown";
            double currentConfidence = 0.0;
            boolean validDetection = false;
            Rect currentRect = null;
            Point currentCenter = null;

            // Process contours to find game elements
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);

                // Skip if contour is too small or too large
                if (area < MIN_CONTOUR_AREA || area > MAX_CONTOUR_AREA) {
                    continue;
                }

                // Approximate the contour with a polygon
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.04 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                // Get the number of vertices in the polygon
                int vertices = (int) approxCurve.total();

                // Get bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Calculate the aspect ratio
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                // Determine shape and orientation
                if (vertices >= 4 && vertices <= 8) {
                    // Calculate how rectangular it is using solidity
                    double rectArea = boundingRect.width * boundingRect.height;
                    double solidity = area / rectArea;

                    if (solidity > 0.7) {
                        // Likely a rectangle or square (game element)

                        // Check orientation based on aspect ratio
                        boolean isHorizontal = Math.abs(aspectRatio - ASPECT_RATIO_HORIZONTAL) < ASPECT_RATIO_TOLERANCE;
                        boolean isVertical = Math.abs(aspectRatio - ASPECT_RATIO_VERTICAL) < ASPECT_RATIO_TOLERANCE;

                        if (isHorizontal) {
                            currentOrientation = "Horizontal";
                        } else if (isVertical) {
                            currentOrientation = "Vertical";
                        } else {
                            currentOrientation = "Unknown";
                        }

                        // Calculate center position and yaw offset
                        double centerX = boundingRect.x + (boundingRect.width / 2.0);
                        double centerY = boundingRect.y + (boundingRect.height / 2.0);
                        currentCenter = new Point(centerX, centerY);

                        double normalizedX = (centerX - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
                        elementYawOffset = normalizedX * (CAMERA_FOV_DEGREES / 2.0);

                        // Set confidence based on how well it matches expected shape
                        currentConfidence = solidity * 100; // 0-100%

                        currentDetectedElement = "GameElement";
                        currentRect = boundingRect;
                        validDetection = true;

                        // We found a valid game element, no need to check other contours
                        break;
                    }
                }
            }

            // Update detection history for stability
            updateDetectionHistory(elementHistory, currentDetectedElement);
            updateDetectionHistory(orientationHistory, currentOrientation);

            if (validDetection) {
                updateRectHistory(rectHistory, currentRect);
                lastDetectedRect = currentRect;
                lastDetectedCenter = currentCenter;
            } else if (rectHistory.size() > 0) {
                // If no detection in this frame but we had recent detections, use the last known position
                // This provides smoother visualization when detection briefly fails
                lastDetectedRect = getMostRecentRect(rectHistory);
                if (lastDetectedRect != null) {
                    lastDetectedCenter = new Point(
                            lastDetectedRect.x + lastDetectedRect.width / 2.0,
                            lastDetectedRect.y + lastDetectedRect.height / 2.0
                    );
                }
            }

            // Get most frequent values from history (temporal smoothing)
            detectedElement = getMostFrequentValue(elementHistory, "None");
            orientationType = getMostFrequentValue(orientationHistory, "Unknown");

            // Draw visualization
            if (lastDetectedRect != null) {
                // Animate highlight thickness for attention-grabbing effect
                highlightThickness += animationDirection;
                if (highlightThickness >= MAX_HIGHLIGHT_THICKNESS || highlightThickness <= MIN_HIGHLIGHT_THICKNESS) {
                    animationDirection *= -1;
                }

                // Draw thick green border around the detected game element
                Imgproc.rectangle(input,
                        lastDetectedRect.tl(),
                        lastDetectedRect.br(),
                        HIGHLIGHT_COLOR,
                        highlightThickness);

                // Draw diagonal corners for enhanced visibility
                int cornerLength = Math.min(lastDetectedRect.width, lastDetectedRect.height) / 4;

                // Top-left corner
                Imgproc.line(input,
                        new Point(lastDetectedRect.x, lastDetectedRect.y),
                        new Point(lastDetectedRect.x + cornerLength, lastDetectedRect.y),
                        HIGHLIGHT_COLOR, highlightThickness);
                Imgproc.line(input,
                        new Point(lastDetectedRect.x, lastDetectedRect.y),
                        new Point(lastDetectedRect.x, lastDetectedRect.y + cornerLength),
                        HIGHLIGHT_COLOR, highlightThickness);

                // Top-right corner
                Imgproc.line(input,
                        new Point(lastDetectedRect.x + lastDetectedRect.width, lastDetectedRect.y),
                        new Point(lastDetectedRect.x + lastDetectedRect.width - cornerLength, lastDetectedRect.y),
                        HIGHLIGHT_COLOR, highlightThickness);
                Imgproc.line(input,
                        new Point(lastDetectedRect.x + lastDetectedRect.width, lastDetectedRect.y),
                        new Point(lastDetectedRect.x + lastDetectedRect.width, lastDetectedRect.y + cornerLength),
                        HIGHLIGHT_COLOR, highlightThickness);

                // Bottom-left corner
                Imgproc.line(input,
                        new Point(lastDetectedRect.x, lastDetectedRect.y + lastDetectedRect.height),
                        new Point(lastDetectedRect.x + cornerLength, lastDetectedRect.y + lastDetectedRect.height),
                        HIGHLIGHT_COLOR, highlightThickness);
                Imgproc.line(input,
                        new Point(lastDetectedRect.x, lastDetectedRect.y + lastDetectedRect.height),
                        new Point(lastDetectedRect.x, lastDetectedRect.y + lastDetectedRect.height - cornerLength),
                        HIGHLIGHT_COLOR, highlightThickness);

                // Bottom-right corner
                Imgproc.line(input,
                        new Point(lastDetectedRect.x + lastDetectedRect.width, lastDetectedRect.y + lastDetectedRect.height),
                        new Point(lastDetectedRect.x + lastDetectedRect.width - cornerLength, lastDetectedRect.y + lastDetectedRect.height),
                        HIGHLIGHT_COLOR, highlightThickness);
                Imgproc.line(input,
                        new Point(lastDetectedRect.x + lastDetectedRect.width, lastDetectedRect.y + lastDetectedRect.height),
                        new Point(lastDetectedRect.x + lastDetectedRect.width, lastDetectedRect.y + lastDetectedRect.height - cornerLength),
                        HIGHLIGHT_COLOR, highlightThickness);

                // Draw center point and target line
                if (lastDetectedCenter != null) {
                    // Center crosshair
                    Imgproc.circle(input, lastDetectedCenter, 5, TARGET_LINE_COLOR, -1);
                    Imgproc.line(input,
                            new Point(lastDetectedCenter.x - 10, lastDetectedCenter.y),
                            new Point(lastDetectedCenter.x + 10, lastDetectedCenter.y),
                            TARGET_LINE_COLOR, 2);
                    Imgproc.line(input,
                            new Point(lastDetectedCenter.x, lastDetectedCenter.y - 10),
                            new Point(lastDetectedCenter.x, lastDetectedCenter.y + 10),
                            TARGET_LINE_COLOR, 2);

                    // Line from center of screen to target
                    Point screenCenter = new Point(FRAME_WIDTH / 2.0, FRAME_HEIGHT / 2.0);
                    Imgproc.line(input, screenCenter, lastDetectedCenter, TARGET_LINE_COLOR, 2);

                    // Draw angle indicator
                    String angleText = String.format("%.1f°", elementYawOffset);
                    Imgproc.putText(input, angleText,
                            new Point((screenCenter.x + lastDetectedCenter.x) / 2,
                                    (screenCenter.y + lastDetectedCenter.y) / 2 - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, TARGET_LINE_COLOR, 2);
                }

                // Add label above detection
                String labelText = detectedElement + " - " + orientationType;
                Imgproc.putText(input, labelText,
                        new Point(lastDetectedRect.x, lastDetectedRect.y - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, HIGHLIGHT_COLOR, 2);
            }

            // Calculate and display FPS
            frameCount++;
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastFpsTimestamp > 1000) { // Every second
                fps = frameCount * 1000.0 / (currentTime - lastFpsTimestamp);
                frameCount = 0;
                lastFpsTimestamp = currentTime;
            }

            // Draw status info
            if (showDebugOverlay) {
                // FPS counter
                Imgproc.putText(input, String.format("FPS: %.1f", fps),
                        new Point(10, FRAME_HEIGHT - 10),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1);

                // Detection status bar
                Imgproc.rectangle(input,
                        new Point(0, 0),
                        new Point(FRAME_WIDTH, 40),
                        new Scalar(0, 0, 0, 180), -1);

                if (validDetection || lastDetectedRect != null) {
                    Imgproc.putText(input, String.format("DETECTED: %s - %s (Yaw: %.1f°)",
                                    detectedElement, orientationType, elementYawOffset),
                            new Point(10, 30),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, HIGHLIGHT_COLOR, 2);
                } else {
                    Imgproc.putText(input, "No Game Element Detected",
                            new Point(10, 30),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 0, 0), 2);
                }

                // Show the threshold image in a corner for debugging
                Mat smallThreshold = new Mat();
                Imgproc.resize(thresholdMat, smallThreshold, new Size(FRAME_WIDTH/4, FRAME_HEIGHT/4));
                Mat threshold3Channel = new Mat();
                Imgproc.cvtColor(smallThreshold, threshold3Channel, Imgproc.COLOR_GRAY2RGB);

                // Overlay at bottom-right corner
                threshold3Channel.copyTo(input.submat(
                        FRAME_HEIGHT - smallThreshold.rows(), FRAME_HEIGHT,
                        FRAME_WIDTH - smallThreshold.cols(), FRAME_WIDTH));

                // Add debug overlay label
                Imgproc.putText(input, "Threshold",
                        new Point(FRAME_WIDTH - smallThreshold.cols(), FRAME_HEIGHT - smallThreshold.rows() - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1);

                smallThreshold.release();
                threshold3Channel.release();
            }

            // Release memory
            grayMat.release();
            blurredMat.release();
            thresholdMat.release();
            cannyOutput.release();
            hierarchy.release();

            return input;
        }

        // Add a new detection to history, maintaining the history size
        private void updateDetectionHistory(LinkedList<String> history, String newValue) {
            if (history.size() >= DETECTION_HISTORY_SIZE) {
                history.removeFirst();
            }
            history.addLast(newValue);
        }

        // Add a new rectangle to history, maintaining the history size
        private void updateRectHistory(LinkedList<Rect> history, Rect newRect) {
            if (newRect == null) return;

            if (history.size() >= DETECTION_HISTORY_SIZE) {
                history.removeFirst();
            }
            history.addLast(newRect.clone());
        }

        // Get most recent rectangle (for visualization when detection temporarily fails)
        private Rect getMostRecentRect(LinkedList<Rect> history) {
            if (history.isEmpty()) {
                return null;
            }
            return history.getLast();
        }

        // Get the most frequent value in the history (for temporal smoothing)
        private String getMostFrequentValue(LinkedList<String> history, String defaultValue) {
            if (history.isEmpty()) {
                return defaultValue;
            }

            Map<String, Integer> frequencyMap = new HashMap<>();
            for (String item : history) {
                frequencyMap.put(item, frequencyMap.getOrDefault(item, 0) + 1);
            }

            String mostFrequent = defaultValue;
            int maxCount = 0;

            for (Map.Entry<String, Integer> entry : frequencyMap.entrySet()) {
                if (entry.getValue() > maxCount) {
                    maxCount = entry.getValue();
                    mostFrequent = entry.getKey();
                }
            }

            return mostFrequent;
        }

        public String getDetectedElement() {
            return detectedElement;
        }

        public double getElementYawOffset() {
            return elementYawOffset;
        }

        public String getOrientationType() {
            return orientationType;
        }

        public double getConfidence() {
            return confidence;
        }

        // Toggle debug overlay
        public void setShowDebugOverlay(boolean show) {
            showDebugOverlay = show;
        }
    }