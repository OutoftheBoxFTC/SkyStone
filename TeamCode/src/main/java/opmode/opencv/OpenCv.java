package opmode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Open Cv")
public class OpenCv extends LinearOpMode {
    private double a, b, c, d;
    OpenCvCamera webcam;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                double[] pixel = input.get(1, 2);
                //bgr
                a=pixel[0];
                b=pixel[1];
                c=pixel[2];
                d=pixel[3];
                return input;
            }
        });
        webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        waitForStart();
        webcam.resumeViewport();
        while (!isStopRequested()){
            long lastTime = System.nanoTime();
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Vars", a + ", " + b + ", " + c + ", " + d);
            long now = System.nanoTime();
            telemetry.addData("fps", 1.0e9/(now-lastTime));
            telemetry.update();
        }
    }
}