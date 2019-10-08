package state;

import hardware.ReadData;
import math.Vector4;

public abstract class DriveState extends LogicState {

    public DriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    public void update(ReadData data) {

    }

    public abstract Vector4 getWheelVelocities();
}