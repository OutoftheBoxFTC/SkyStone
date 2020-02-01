package State;

import HardwareSystems.SensorData;
import Motion.MecanumSystem;
import Motion.VelocitySystem;
import math.Vector3;
import math.Vector4;

public abstract class VelocityDriveState extends DriveState {
    public VelocityDriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    public Vector4 getWheelVelocities(SensorData sensors) {
        return MecanumSystem.translate(getVelocities());
    }

    public abstract Vector3 getVelocities();
}
