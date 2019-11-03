package Motion;

import Hardware.HardwareData;
import Hardware.SensorData;
import State.DriveState;
import State.StateMachine;
import math.Vector4;

public class DriveSystem extends DriveState {
    public DriveSystem(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    public Vector4 getWheelVelocities() {
        return null;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardware) {

    }
}
