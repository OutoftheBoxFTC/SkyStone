package opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import math.Vector2;
import math.Vector3;

public class DetectionPipeline extends OpenCvPipeline {
    private int detection;
    private Vector3 position;

    private Vector2 skystone1, skystone2;

    public DetectionPipeline(Vector3 position){
        this.position = position;
    }

    @Override
    public Mat processFrame(Mat input) {

        return input;
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

    public int getDetectedPosition(){
        return 0;//TODO finish
    }
}
