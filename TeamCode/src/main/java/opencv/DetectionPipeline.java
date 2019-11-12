package opencv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
    private Point[] points;
    private int pos;
    public DetectionPipeline(){
        points = new Point[]{new Point(358, 359), new Point(429, 383), new Point(358, 209), new Point(429, 235)};
    }

    @Override
    public Mat processFrame(Mat input) {
        double[] color1 = avgColor(input, points[0], points[1]), color2 = avgColor(input, points[2], points[3]);
        double avg1 = (color1[0]+color1[1]+color1[2])/3, avg2 = (color2[0]+color1[1]+color1[2])/3;
        if(Math.abs(avg1-avg2)<20){
            pos = 2;
        } else if(avg1<avg2){
            pos = 0;
        } else {
            pos = 1;
        }
        Imgproc.rectangle(input, points[0], points[1], new Scalar(255, 0, 0));//1: red
        Imgproc.rectangle(input, points[2], points[3], new Scalar(0, 0, 255));//2: blue
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
        return pos;
    }
}
