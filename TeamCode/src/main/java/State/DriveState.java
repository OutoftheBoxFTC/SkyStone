package State;

import HardwareSystems.SensorData;
import math.Vector4;

public abstract class DriveState extends LogicState {

    public DriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    public abstract Vector4 getWheelVelocities(SensorData sensors);
}