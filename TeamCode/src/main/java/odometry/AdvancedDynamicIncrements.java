package odometry;

import math.Vector3;

@Deprecated
public class AdvancedDynamicIncrements extends SimpleDynamicIncrements {
    private Vector3 dynamicVelocityIncrements, previousVelocity;
    public AdvancedDynamicIncrements(Vector3 dynamicRobotIncrements, Vector3 dynamicVelocityIncrements, Vector3 previousVelocity) {
        super(dynamicRobotIncrements);
        this.dynamicVelocityIncrements = dynamicVelocityIncrements;
        this.previousVelocity = previousVelocity;
    }

    public Vector3 getDynamicVelocityIncrements() {
        return dynamicVelocityIncrements;
    }

    public Vector3 getPreviousVelocity() {
        return previousVelocity;
    }
}