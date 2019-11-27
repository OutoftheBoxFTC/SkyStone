package Odometer;

import Hardware.SensorData;
import math.Vector2;
import math.Vector3;

public class SimpleOdometer {
    public double translationFactor;
    double x = 0;
    double y = 0;
    long lastTime;
    Vector3 lastCoords;
    Vector2 offsets;
    protected final static double RADIUS_RATIO = (290.49/430.43);
    Vector3 position, velocity;
    public SimpleOdometer(double translationFactor, Vector3 position, Vector3 velocity){
        this.translationFactor = translationFactor;
        this.position = position;
        this.velocity = velocity;
    }

    public void start(SensorData data){
        double forward = ((data.getLeft() + data.getRight())/2);
        double rotDiff = data.getRight() - forward;
        double aux = (data.getAux() - (rotDiff * RADIUS_RATIO));
        lastCoords = new Vector3(forward, data.getAux(), data.getGyro());
        lastTime = System.currentTimeMillis();
        position.set(new Vector3(0, 0, Math.toDegrees(data.getGyro())));
    }

    public void update(SensorData data){
        double forward = ((data.getLeft() + data.getRight())/2);
        double rotDiff = data.getRight() - forward;
        double rotInc = data.getGyro() - lastCoords.getC();
        double aux = ((data.getAux()) - (rotInc * 1735));
        double forwardInc = forward - lastCoords.getA();
        double auxInc = aux - lastCoords.getB();
        double r = (Math.sqrt((forwardInc * forwardInc) + (auxInc * auxInc)));
        if(r < 0.05){
            r = 0;
        }
        x += r * Math.cos(data.getGyro() + Math.atan2(forwardInc,auxInc));
        y += r * Math.sin(data.getGyro() + Math.atan2(forwardInc,auxInc));
        position.set(new Vector3(x * translationFactor, y * translationFactor, Math.toDegrees(data.getGyro())));
        lastCoords.set(new Vector3(forward, aux, data.getGyro()));
    }
}
