package Motion;

import Hardware.HardwareData;
import Hardware.SensorData;
import State.DriveState;
import State.StateMachine;
import math.Vector3;
import math.Vector4;

public class TurnCorrectionVector extends DriveState {
    double targetAngle, kp, correction;
    Vector3 position;
    public TurnCorrectionVector(StateMachine stateMachine, double kp, double angle, Vector3 position){
        super(stateMachine);
        this.kp = kp;
        targetAngle = angle;
        this.position = position;
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
        if(Math.abs(correction) < 0.15){
            correction = (correction/Math.abs(correction)) * 0.15;
        }
    }

    @Override
    public Vector4 getWheelVelocities(SensorData data) {
        return MecanumSystem.translate(new Vector3(0, 0, correction));
    }

}
