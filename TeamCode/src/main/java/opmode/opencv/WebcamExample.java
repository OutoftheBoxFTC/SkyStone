package opmode.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Webcam Test")
@Disabled
public class WebcamExample extends LinearOpMode {
    OpenCvCamera webcam;
    int x1, y1, x2, y2;
    double b, g, r;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Point p1 = new Point(x1, y1);
                Point p2 = new Point(x2, y2);
                Imgproc.rectangle(input, p1, p2, new Scalar(255, 255, 255));
                double[] avgColor = avgColor(input, p1, p2);
                b = avgColor[0];
                g = avgColor[1];
                r = avgColor[2];

                return input;
            }
        });
        webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                y1--;
            } else if(gamepad1.dpad_down){
                y1++;
            }
            if(gamepad1.dpad_left){
                x1--;
            } else if(gamepad1.dpad_right){
                x1++;
            }

            if(gamepad2.dpad_up){
                y2--;
            } else if(gamepad2.dpad_down){
                y2++;
            }
            if(gamepad2.dpad_left){
                x2--;
            } else if(gamepad2.dpad_right){
                x2++;
            }
            telemetry.addData("x1", x1);
            telemetry.addData("x2", x2);
            telemetry.addData("y1", y1);
            telemetry.addData("y2", y2);
            telemetry.addData("b", b);
            telemetry.addData("g", g);
            telemetry.addData("r", r);
            telemetry.update();
            sleep(10*(gamepad1.a?2:1));
        }
    }

    private double[] avgColor(Mat img, Point p1, Point p2){
        double[] color = new double[3];
        double area = (p1.x-p2.x)*(p1.y-p2.y);
        for(int x = (int) p2.x; x < p1.x; x++){
            for(int y = (int) p2.y; y < p1.y; y++){
                double[] pixel = img.get(y, x);
                color[0] += pixel[0]/area;
                color[1] += pixel[1]/area;
                color[2] += pixel[2]/area;
            }
        }
        return color;
    }
}