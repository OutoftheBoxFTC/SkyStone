package Motion;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.DriveState;
import State.StateMachine;
import math.Vector3;
import math.Vector4;

public class TurnCorrectionVector extends DriveState {
    double targetAngle, kp, correction, power;
    Vector3 position;
    public TurnCorrectionVector(StateMachine stateMachine, double kp, double angle, Vector3 position, double power){
        super(stateMachine);
        this.kp = kp;
        targetAngle = angle;
        this.position = position;
        this.power = power;
    }
    @Override
    public void update(SensorData sensors, HardwareData hardware) {
        correction = (targetAngle - (Math.toDegrees(sensors.getGyro())));
        if(correction > 180){
            correction = (targetAngle - (360 + Math.toDegrees(sensors.getGyro())));
        }else if(correction < -180){
            correction = ((360 + targetAngle) - Math.toDegrees(sensors.getGyro()));
        }
        correction *= kp;
        if(Math.abs(correction) < power){
            correction = (correction/Math.abs(correction)) * power;
        }
    }

    @Override
    public Vector4 getWheelVelocities(SensorData data) {
        return MecanumSystem.translate(new Vector3(0, 0, correction));
    }

}
