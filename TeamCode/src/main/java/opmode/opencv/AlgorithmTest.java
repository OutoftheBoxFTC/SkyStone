package opmode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Algorithm Test")
public class AlgorithmTest extends LinearOpMode {
    OpenCvCamera webcam;
    private Point[] color1, color2;
    double[] ratio;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        color1 = new Point[]{new Point(92, 81), new Point(438, 230)};
        color2 = new Point[]{new Point(92, 450), new Point(438, 600)};
        ratio = new double[3];
        webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                double[] value1 = avgColor(input, color1[0], color1[1]);
                double[] value2 = avgColor(input, color2[0], color2[1]);
                ratio[0] = value1[0]/value2[0];
                ratio[1] = value1[1]/value2[1];
                ratio[2] = value1[2]/value2[2];
                Imgproc.rectangle(input, color1[0], color1[1], new Scalar(255, 255, 255));
                Imgproc.rectangle(input, color2[0], color2[1], new Scalar(255, 0, 0));
                return input;
            }
        });
        webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("ratio1", ratio[0]);
            telemetry.addData("ratio2", ratio[1]);
            telemetry.addData("ratio3", ratio[2]);
            telemetry.update();
        }
    }

    private double[] avgColor(Mat img, Point p1, Point p2){
        double[] color = new double[3];
        double area = (p2.x-p1.x)*(p2.y-p1.y);
        for(int x = (int) p1.x; x < p2.x; x++){
            for(int y = (int) p1.y; y < p2.y; y++){
                double[] pixel = img.get(y, x);
                color[0] += pixel[0]/area;
                color[1] += pixel[1]/area;
                color[2] += pixel[2]/area;
            }
        }
        return color;
    }
}