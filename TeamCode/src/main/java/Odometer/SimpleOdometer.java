package Odometer;

import com.qualcomm.robotcore.util.RobotLog;

import debug.SmartTelemetry;
import hardware.ReadData;
import math.Vector3;

public class SimpleOdometer {
    public double translationFactor;
    double x = 0;
    double y = 0;
    long lastTime;
    Vector3 lastCoords;
    protected final static double RADIUS_RATIO = (290.49/430.43);
    Vector3 position, velocity;
    SmartTelemetry telemetry;
    public SimpleOdometer(double translationFactor, Vector3 position, Vector3 velocity, SmartTelemetry telemetry){
        this.translationFactor = translationFactor;
        this.position = position;
        this.velocity = velocity;
        this.telemetry = telemetry;
    }

    public void start(ReadData data){
        double forward = ((data.getLeft() + data.getRight())/2);
        double rotDiff = data.getRight() - forward;
        double aux = (data.getAux() - (rotDiff * RADIUS_RATIO));
        lastCoords = new Vector3(forward, aux, data.getGyro());
        lastTime = System.currentTimeMillis();
        position.set(new Vector3(0, 0, Math.toDegrees(data.getGyro())));
    }

    public void update(ReadData data){
        double forward = ((data.getLeft() + data.getRight())/2);
        double rotDiff = data.getRight() - forward;
        double rotInc = data.getGyro() - lastCoords.getC();
        double aux = (data.getAux() - (rotInc * 0.0423583858));
        double forwardInc = forward - lastCoords.getA();
        double auxInc = aux - lastCoords.getB();
        double r = (Math.sqrt((forwardInc * forwardInc) + (auxInc * auxInc)));
        if(r < 0.05){
            r = 0;
        }
        x += r * Math.cos(data.getGyro() + Math.atan2(forwardInc,auxInc));
        y += r * Math.sin(data.getGyro() + Math.atan2(forwardInc,auxInc));
        telemetry.setHeader("Angle", Math.toDegrees(Math.atan2(forwardInc, auxInc)));
        position.set(new Vector3(x * translationFactor, y * translationFactor, Math.toDegrees(data.getGyro())));
        lastCoords.set(new Vector3(forward, aux, data.getGyro()));
    }
}
