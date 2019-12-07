package cv;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import math.Vector2;
import math.Vector3;

public class PlatformDetection extends OpenCvPipeline {

    //assumes pressed against platform long end
    private static final Vector3 cameraHomePosition;
    private static final Vector2 camRotation;

    static {
        cameraHomePosition = new Vector3(0, 0, 0);
        camRotation = new Vector2();
        //TODO lmao find me
    }

    private Vector3 liftOffset;

    public PlatformDetection(){
        liftOffset = Vector3.ZERO();
    }

    @Override
    public Mat processFrame(Mat input) {
        Vector3 camPosition = cameraHomePosition.add(liftOffset);

    }
}
