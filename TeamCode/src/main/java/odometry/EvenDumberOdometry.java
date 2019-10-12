package odometry;

import hardware.ReadData;
import math.Vector3;

public class EvenDumberOdometry {
    double translationFactor;
    double x = 0;
    double y = 0;
    protected final static double RADIUS_RATIO = (290.49/430.43);
    public EvenDumberOdometry(double translationFactor){
        this.translationFactor = translationFactor;
    }
    public Vector3 getPosition(ReadData data){
        double forward = (data.getLeft() + data.getRight())/2;
        double rotDiff = data.getRight() - forward;
        double aux = data.getAux() - (rotDiff * RADIUS_RATIO);
        double r = Math.sqrt((forward * forward) + (aux * aux));
        x += r * Math.cos(data.getGyro() + Math.atan(forward/aux));
        y += r * Math.sin(data.getGyro() + Math.atan(forward/aux));
        return new Vector3(x, y, data.getGyro());
    }
}
