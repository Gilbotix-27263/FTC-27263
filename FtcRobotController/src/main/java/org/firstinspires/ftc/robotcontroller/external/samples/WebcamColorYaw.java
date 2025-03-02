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
import java.util.LinkedList;
import java.util.List;

@TeleOp(name = "Webcam Color Yaw", group = "Main")
public class WebcamColorYaw extends LinearOpMode {
    private OpenCvCamera webcam;
    private IMU imu;
    private Servo intake;
    private Servo intakelr;
    private EnhancedShapeDetectionPipeline colorPipeline;
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
        colorPipeline = new EnhancedShapeDetectionPipeline();
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

class EnhancedShapeDetectionPipeline extends OpenCvPipeline {
    private String detectedElement = "None";
    private double elementYawOffset = 0.0;
    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;
    private String orientationType = "Unknown";
    private double confidence = 0.0;

    // Visualization parameters
    private boolean showDebugOverlay = true;
    private Point lastDetectedCenter = null;
    private Rect lastDetectedRect = null;

    // Detection highlight colors
    private static final Scalar HIGHLIGHT_COLOR = new Scalar(0, 255, 0); // Green
    private static final Scalar TARGET_LINE_COLOR = new Scalar(255, 0, 0); // Red
    private static final Scalar TEXT_COLOR = new Scalar(255, 255, 255); // White

    // Camera parameters
    private static final double CAMERA_FOV_DEGREES = 60.0;

    // Game element dimensions (in inches)
    private static final double ELEMENT_WIDTH_INCHES = 3.5;
    private static final double ELEMENT_HEIGHT_INCHES = 1.5;

    // Expected aspect ratios for different orientations
    private static final double ASPECT_RATIO_HORIZONTAL = ELEMENT_WIDTH_INCHES / ELEMENT_HEIGHT_INCHES;
    private static final double ASPECT_RATIO_VERTICAL = ELEMENT_HEIGHT_INCHES / ELEMENT_WIDTH_INCHES;
    private static final double ASPECT_RATIO_TOLERANCE = 0.5;

    // Tracking history for stable detection
    private static final int DETECTION_HISTORY_SIZE = 10;
    private final LinkedList<String> elementHistory = new LinkedList<>();
    private final LinkedList<String> orientationHistory = new LinkedList<>();
    private final LinkedList<Point> centerHistory = new LinkedList<>();

    // Detection parameters
    private static final double MIN_CONTOUR_AREA = 400;
    private static final double MAX_CONTOUR_AREA = FRAME_WIDTH * FRAME_HEIGHT / 4;

    // Animation parameters for highlight effect
    private int highlightThickness = 2;
    private int animationDirection = 1;
    private static final int MIN_HIGHLIGHT_THICKNESS = 2;
    private static final int MAX_HIGHLIGHT_THICKNESS = 5;

    // HSV color ranges for game element - adjust based on your game elements
    private static final Scalar LOWER_COLOR = new Scalar(20, 100, 100);   // Lower HSV bound
    private static final Scalar UPPER_COLOR = new Scalar(30, 255, 255);   // Upper HSV bound

    @Override
    public Mat processFrame(Mat input) {
        // Create working matrices
        Mat hsvMat = new Mat();
        Mat colorMask = new Mat();
        Mat blurredMat = new Mat();
        Mat hierarchy = new Mat();

        // Convert to HSV color space for better color detection
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Filter for game element color
        Core.inRange(hsvMat, LOWER_COLOR, UPPER_COLOR, colorMask);

        // Apply blur to reduce noise
        Imgproc.GaussianBlur(colorMask, blurredMat, new Size(5, 5), 0);

        // Apply morphological operations to clean up the image
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel, new Point(-1, -1), 2);
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_OPEN, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(blurredMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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

            // Get bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(contour);

            // Calculate aspect ratio and solidity (shape metrics)
            double aspectRatio = (double) boundingRect.width / boundingRect.height;
            double rectArea = boundingRect.width * boundingRect.height;
            double solidity = area / rectArea;

            // Check if contour is rectangular enough and has enough area
            if (solidity > 0.7 && area > MIN_CONTOUR_AREA) {
                // Determine orientation based on aspect ratio
                if (Math.abs(aspectRatio - ASPECT_RATIO_HORIZONTAL) < Math.abs(aspectRatio - ASPECT_RATIO_VERTICAL)) {
                    currentOrientation = "Horizontal";
                } else {
                    currentOrientation = "Vertical";
                }

                // Calculate center position and yaw offset
                double centerX = boundingRect.x + (boundingRect.width / 2.0);
                double centerY = boundingRect.y + (boundingRect.height / 2.0);
                currentCenter = new Point(centerX, centerY);

                double normalizedX = (centerX - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
                elementYawOffset = normalizedX * (CAMERA_FOV_DEGREES / 2.0);

                // Set confidence based on solidity and aspect ratio match
                double aspectRatioMatch;
                if ("Horizontal".equals(currentOrientation)) {
                    aspectRatioMatch = 1.0 - Math.min(1.0, Math.abs(aspectRatio - ASPECT_RATIO_HORIZONTAL) / ASPECT_RATIO_HORIZONTAL);
                } else {
                    aspectRatioMatch = 1.0 - Math.min(1.0, Math.abs(aspectRatio - ASPECT_RATIO_VERTICAL) / ASPECT_RATIO_VERTICAL);
                }

                currentConfidence = (solidity * 0.5 + aspectRatioMatch * 0.5) * 100; // 0-100%

                if (currentConfidence > 60) {
                    currentDetectedElement = "GameElement";
                    currentRect = boundingRect;
                    validDetection = true;
                    break;
                }
            }
        }

        // Update detection history for stability
        updateDetectionHistory(elementHistory, currentDetectedElement);
        updateDetectionHistory(orientationHistory, currentOrientation);
        if (currentCenter != null) {
            updatePointHistory(centerHistory, currentCenter);
        }

        // Get smoothed position by averaging recent detections
        Point smoothedCenter = null;
        if (!centerHistory.isEmpty()) {
            smoothedCenter = getAveragePoint(centerHistory);
            if (smoothedCenter != null) {
                double normalizedX = (smoothedCenter.x - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
                elementYawOffset = normalizedX * (CAMERA_FOV_DEGREES / 2.0);
            }
        }

        // Update visualization parameters
        if (validDetection) {
            lastDetectedRect = currentRect;
            lastDetectedCenter = smoothedCenter != null ? smoothedCenter : currentCenter;
        } else if (!centerHistory.isEmpty()) {
            // Use history for smoother visualization
            lastDetectedCenter = smoothedCenter;

            // Reconstruct rectangle from center point if we have a recent one
            if (lastDetectedRect != null && smoothedCenter != null) {
                int width = lastDetectedRect.width;
                int height = lastDetectedRect.height;
                lastDetectedRect = new Rect(
                        (int)(smoothedCenter.x - width/2),
                        (int)(smoothedCenter.y - height/2),
                        width, height
                );
            }
        }

        // Get most frequent values from history (temporal smoothing)
        detectedElement = getMostFrequentValue(elementHistory, "None");
        orientationType = getMostFrequentValue(orientationHistory, "Unknown");
        confidence = currentConfidence;

        // Draw visualization
        if (lastDetectedRect != null && "GameElement".equals(detectedElement)) {
            // Animate highlight thickness
            highlightThickness += animationDirection;
            if (highlightThickness >= MAX_HIGHLIGHT_THICKNESS || highlightThickness <= MIN_HIGHLIGHT_THICKNESS) {
                animationDirection *= -1;
            }

            // Draw rectangle around the detected game element
            Imgproc.rectangle(input, lastDetectedRect.tl(), lastDetectedRect.br(), HIGHLIGHT_COLOR, highlightThickness);

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
            String labelText = detectedElement + " - " + orientationType + " (" + String.format("%.0f%%", confidence) + ")";
            Imgproc.putText(input, labelText,
                    new Point(lastDetectedRect.x, lastDetectedRect.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, HIGHLIGHT_COLOR, 2);
        }

        // Draw status info overlay
        if (showDebugOverlay) {
            // Status bar background
            Imgproc.rectangle(input,
                    new Point(0, 0),
                    new Point(FRAME_WIDTH, 40),
                    new Scalar(0, 0, 0, 180), -1);

            if ("GameElement".equals(detectedElement)) {
                Imgproc.putText(input, String.format("DETECTED: %s - %s (Yaw: %.1f°)",
                                detectedElement, orientationType, elementYawOffset),
                        new Point(10, 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, HIGHLIGHT_COLOR, 2);
            } else {
                Imgproc.putText(input, "No Game Element Detected",
                        new Point(10, 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 0, 0), 2);
            }

            // Show the color mask in a corner for debugging
            Mat smallMask = new Mat();
            Imgproc.resize(colorMask, smallMask, new Size(FRAME_WIDTH/4, FRAME_HEIGHT/4));
            Mat mask3Channel = new Mat();
            Imgproc.cvtColor(smallMask, mask3Channel, Imgproc.COLOR_GRAY2RGB);

            // Overlay at bottom-right corner
            mask3Channel.copyTo(input.submat(
                    FRAME_HEIGHT - smallMask.rows(), FRAME_HEIGHT,
                    FRAME_WIDTH - smallMask.cols(), FRAME_WIDTH));

            // Add debug overlay label
            Imgproc.putText(input, "Color Mask",
                    new Point(FRAME_WIDTH - smallMask.cols(), FRAME_HEIGHT - smallMask.rows() - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1);

            // Release temporary mats
            smallMask.release();
            mask3Channel.release();
        }

        // Release memory
        hsvMat.release();
        colorMask.release();
        blurredMat.release();
        hierarchy.release();

        return input;
    }

    // Utility methods for history tracking and smoothing
    private void updateDetectionHistory(LinkedList<String> history, String newValue) {
        if (history.size() >= DETECTION_HISTORY_SIZE) {
            history.removeFirst();
        }
        history.addLast(newValue);
    }

    private void updatePointHistory(LinkedList<Point> history, Point newPoint) {
        if (history.size() >= DETECTION_HISTORY_SIZE) {
            history.removeFirst();
        }
        history.addLast(newPoint.clone());
    }

    private Point getAveragePoint(LinkedList<Point> history) {
        if (history.isEmpty()) {
            return null;
        }

        double sumX = 0, sumY = 0;
        for (Point p : history) {
            sumX += p.x;
            sumY += p.y;
        }

        return new Point(sumX / history.size(), sumY / history.size());
    }

    private String getMostFrequentValue(LinkedList<String> history, String defaultValue) {
        if (history.isEmpty()) {
            return defaultValue;
        }

        // Count occurrences
        java.util.Map<String, Integer> counts = new java.util.HashMap<>();
        for (String s : history) {
            counts.put(s, counts.getOrDefault(s, 0) + 1);
        }

        // Find most frequent
        String mostFrequent = defaultValue;
        int maxCount = 0;

        for (java.util.Map.Entry<String, Integer> entry : counts.entrySet()) {
            if (entry.getValue() > maxCount) {
                maxCount = entry.getValue();
                mostFrequent = entry.getKey();
            }
        }

        return mostFrequent;
    }

    // Accessor methods
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

    public void setShowDebugOverlay(boolean show) {
        showDebugOverlay = show;
    }
}