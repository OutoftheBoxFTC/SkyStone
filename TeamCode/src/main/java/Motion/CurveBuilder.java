package Motion;

import java.util.ArrayList;

import State.StateMachine;
import math.Vector2;
import math.Vector3;

public class CurveBuilder {
    double speed, radius, slowdownDistance, minPower, angle, startTurn, initialAngle;
    boolean optimizeAngle;
    Vector3 position;
    StateMachine stateMachine;
    ArrayList<Vector2> splinePoints;
    public CurveBuilder(StateMachine stateMachine, Vector3 position){
        speed = 1;
        splinePoints = new ArrayList<>();
        radius = 5;
        this.stateMachine = stateMachine;
        this.position = position;
        slowdownDistance = 10;
        minPower = 0.4;
        angle = 0;
        this.startTurn = 100000000;
        this.initialAngle = 0;
    }

    public CurveBuilder newCurve(){
        this.speed = 1;
        this.splinePoints.clear();
        this.radius = 5;
        slowdownDistance = 10;
        minPower = 0.4;
        angle = 0;
        this.startTurn = 100000000;
        this.initialAngle = 0;
        return this;
    }

    public CurveBuilder setSpeed(double speed){
        this.speed = speed;
        return this;
    }

    public CurveBuilder setRadius(double radius) {
        this.radius = radius;
        return this;
    }

    public CurveBuilder setSpline(Vector2 spline){
        this.splinePoints.add(spline);
        return this;
    }

    public CurveBuilder setSlowdownDistance(double slowdownDistance){
        this.slowdownDistance = slowdownDistance;
        return this;
    }

    public CurveBuilder setInitialAngle(double angle){
        this.initialAngle = angle;
        return this;
    }

    public CurveBuilder setMinPower(double minPower){
        this.minPower = minPower;
        return this;
    }

    public CurveBuilder setAngle(double angle){
        this.angle = angle;
        return this;
    }

    public CurveBuilder setTurnStartDistance(double distance){
        this.startTurn = distance;
        return this;
    }

    public CurveBuilder optimiseAngle(){
        this.optimizeAngle = true;
        return this;
    }

    public CurveCorrectionSystem complete(){
        return new CurveCorrectionSystem(stateMachine, position, splinePoints, speed, radius, slowdownDistance, minPower, angle, startTurn, initialAngle, optimizeAngle);
    }
}
