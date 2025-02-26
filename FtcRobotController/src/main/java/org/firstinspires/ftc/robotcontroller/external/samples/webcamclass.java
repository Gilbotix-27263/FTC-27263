package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

@TeleOp(group = "Main")
public class webcamclass extends LinearOpMode {
    private OpenCvCamera webcam;
    private IMU imu;
    private ColorDetectionPipeline colorPipeline;
    private static final double CAMERA_FOV_DEGREES = 60.0;

    @Override
    public void runOpMode() {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize webcam
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
            // Get robot's yaw from IMU
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double robotYaw = orientation.getYaw(AngleUnit.DEGREES);

            // Get detected color and its x-position
            String detectedColor = colorPipeline.getDetectedColor();
            double sampleYaw = robotYaw + colorPipeline.getSampleYawOffset();

            // Display telemetry
            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Sample Yaw", "%.2f degrees", sampleYaw);
            telemetry.update();
        }
    }

    static class ColorDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "None";
        private double sampleYawOffset = 0.0;
        private static final int FRAME_WIDTH = 640;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Scalar lowerRed = new Scalar(0, 120, 70);
            Scalar upperRed = new Scalar(10, 255, 255);
            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            Mat redMask = new Mat();
            Mat blueMask = new Mat();
            Mat yellowMask = new Mat();

            Core.inRange(hsvMat, lowerRed, upperRed, redMask);
            Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);
            Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);

            int redCount = Core.countNonZero(redMask);
            int blueCount = Core.countNonZero(blueMask);
            int yellowCount = Core.countNonZero(yellowMask);

            if (redCount > blueCount && redCount > yellowCount) {
                detectedColor = "Red";
                sampleYawOffset = calculateYawOffset(redMask);
            } else if (blueCount > redCount && blueCount > yellowCount) {
                detectedColor = "Blue";
                sampleYawOffset = calculateYawOffset(blueMask);
            } else if (yellowCount > redCount && yellowCount > blueCount) {
                detectedColor = "Yellow";
                sampleYawOffset = calculateYawOffset(yellowMask);
            } else {
                detectedColor = "None";
                sampleYawOffset = 0.0;
            }

            redMask.release();
            blueMask.release();
            yellowMask.release();
            hsvMat.release();

            return input;
        }

        private double calculateYawOffset(Mat mask) {
            Rect boundingRect = Imgproc.boundingRect(mask);
            double centerX = boundingRect.x + (boundingRect.width / 2.0);
            double normalizedX = (centerX - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
            return normalizedX * (CAMERA_FOV_DEGREES / 2.0);
        }

        public String getDetectedColor() {
            return detectedColor;
        }

        public double getSampleYawOffset() {
            return sampleYawOffset;
        }
    }
}
