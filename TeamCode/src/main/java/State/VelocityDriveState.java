package State;

import Hardware.SensorData;
import Motion.MecanumSystem;
import Motion.VelocitySystem;
import math.Vector3;
import math.Vector4;

public abstract class VelocityDriveState extends DriveState {
    private VelocitySystem system;
    public VelocityDriveState(StateMachine stateMachine, VelocitySystem system) {
        super(stateMachine);
        this.system = system;
    }

    @Override
    public Vector4 getWheelVelocities(SensorData sensors) {
        system.setTargetVelocity(getVelocities());
        return MecanumSystem.translate(system.update());
    }

    public abstract Vector3 getVelocities();
}
