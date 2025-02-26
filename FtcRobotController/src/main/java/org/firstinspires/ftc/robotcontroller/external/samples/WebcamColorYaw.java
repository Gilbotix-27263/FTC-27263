package org.firstinspires.ftc.robotcontroller.external.samples;

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
                } else if ("Horizontal".equals(orientationType)){
                    intake.setPosition(0);
                    intakelr.setPosition(1.0); // Rotate until vertical
                    sleep(500); // Allow time for rotation
                    intakelr.setPosition(0.5); // Stop rotation
                    intake.setPosition(1.0); // Close intake
                }
                else {
                    intake.setPosition(0);
                }
            }
        }
    }

    static class ColorDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "None";
        private double sampleYawOffset = 0.0;
        private static final int FRAME_WIDTH = 640;
        private String orientationType = "Unknown";

        @Override
        public Mat processFrame(Mat input) {
            Mat rgbMat = new Mat();
            Imgproc.cvtColor(input, rgbMat, Imgproc.COLOR_BGR2RGB); // Convert BGR to RGB

            Mat maskRed = new Mat();
            Mat maskBlue = new Mat();
            Mat maskYellow = new Mat();

            Core.inRange(rgbMat, new Scalar(200, 0, 0), new Scalar(255, 80, 80), maskRed); // Red Range
            Core.inRange(rgbMat, new Scalar(0, 0, 200), new Scalar(80, 80, 255), maskBlue); // Blue Range
            Core.inRange(rgbMat, new Scalar(180, 180, 0), new Scalar(255, 255, 100), maskYellow); // Yellow Range

            Mat mask = new Mat();
            Core.bitwise_or(maskRed, maskBlue, mask);
            Core.bitwise_or(mask, maskYellow, mask);

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                double maxArea = Imgproc.contourArea(largestContour);

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                Rect boundingRect = Imgproc.boundingRect(largestContour);
                Imgproc.rectangle(rgbMat, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);
                double centerX = boundingRect.x + (boundingRect.width / 2.0);
                double normalizedX = (centerX - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
                sampleYawOffset = normalizedX * (CAMERA_FOV_DEGREES / 2.0);
                orientationType = boundingRect.width > boundingRect.height ? "Horizontal" : "Vertical";
            }

            maskRed.release();
            maskBlue.release();
            maskYellow.release();
            mask.release();
            rgbMat.release();
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
