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

@TeleOp(name = "Webcam Shape Yaw", group = "Main")
public class WebcamShapeYaw extends LinearOpMode {
    private OpenCvCamera webcam;
    private IMU imu;
    private Servo intake;
    private Servo intakelr;
    private ShapeDetectionPipeline shapePipeline;
    private static final double CAMERA_FOV_DEGREES = 60.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        intake = hardwareMap.get(Servo.class, "intake");
        intakelr = hardwareMap.get(Servo.class, "intakelr");

        intake.setPosition(0.0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        shapePipeline = new ShapeDetectionPipeline();
        webcam.setPipeline(shapePipeline);

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

            double sampleYawOffset = shapePipeline.getSampleYawOffset();
            double sampleYaw = robotYaw + sampleYawOffset;
            String orientationType = shapePipeline.getOrientationType();

            telemetry.addData("Sample Yaw Offset", "%.2f degrees", sampleYawOffset);
            telemetry.addData("Sample Yaw", "%.2f degrees", sampleYaw);
            telemetry.addData("Object Orientation", orientationType);
            telemetry.update();

            if (Math.abs(sampleYawOffset) < 5) { // Sample is centered
                if ("Vertical".equals(orientationType)) {
                    intake.setPosition(1.0);
                } else if ("Horizontal".equals(orientationType)) {
                    intake.setPosition(0);
                    intakelr.setPosition(1.0);
                    sleep(500);
                    intakelr.setPosition(0.5);
                    intake.setPosition(1.0);
                } else {
                    intake.setPosition(0);
                }
            }
        }
    }

    static class ShapeDetectionPipeline extends OpenCvPipeline {
        private double sampleYawOffset = 0.0;
        private static final int FRAME_WIDTH = 640;
        private String orientationType = "Unknown";

        @Override
        public Mat processFrame(Mat input) {
            Mat grayMat = new Mat();
            Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_RGB2GRAY);
            Imgproc.GaussianBlur(grayMat, grayMat, new Size(5, 5), 0);
            Imgproc.Canny(grayMat, grayMat, 50, 150);

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(grayMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);
                double centerX = boundingRect.x + (boundingRect.width / 2.0);
                double normalizedX = (centerX - (FRAME_WIDTH / 2.0)) / (FRAME_WIDTH / 2.0);
                sampleYawOffset = normalizedX * (CAMERA_FOV_DEGREES / 2.0);
                orientationType = boundingRect.width > boundingRect.height ? "Horizontal" : "Vertical";
            }

            grayMat.release();
            return input;
        }

        public double getSampleYawOffset() {
            return sampleYawOffset;
        }

        public String getOrientationType() {
            return orientationType;
        }
    }
}
