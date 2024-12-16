package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "Webcam Test", group = "Samples")
public class WebcamTest extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // Get the camera monitor view ID for displaying the stream
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Create the webcam instance
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set a simple pipeline for processing frames
        webcam.setPipeline(new SamplePipeline());

        // Open the camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming video at a resolution of 640x480
                webcam.startStreaming(640, 480);
                telemetry.addData("Status", "Webcam Opened");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam Error", "Error Code: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for the game to start
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Main OpMode loop
        while (opModeIsActive()) {
            // Display telemetry information about the camera stream
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            telemetry.addData("Total Frame Time (ms)", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline Time (ms)", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical Max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }

        // Stop the camera when the OpMode ends
        webcam.stopStreaming();
    }

    // Simple pipeline for processing frames
    static class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Example: Convert the input frame to grayscale
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
            return input; // Return the processed frame
        }
    }
}