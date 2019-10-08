package Odometer;

import math.Vector3;

public class SimpleDynamicIncrements {
    private final Vector3 dynamicRobotIncrements;
    public SimpleDynamicIncrements(Vector3 dynamicRobotIncrements){
        this.dynamicRobotIncrements = dynamicRobotIncrements;
    }

    public Vector3 getDynamicRobotIncrements() {
        return dynamicRobotIncrements;
    }
}