package Motion;

import HardwareSystems.SensorData;
import math.Vector2;
import math.Vector3;

public class VelocitySystem {
    Vector3 velocity, rawVelocity, tempVelocity;
    double kp = 0.1;
    boolean isStarted = false;
    public VelocitySystem(){
        velocity = Vector3.ZERO();
        rawVelocity = Vector3.ZERO();
    }

    public void start(Vector3 velocity){
        this.rawVelocity.set(velocity);
        this.isStarted = true;
    }


    public Vector3 update(Vector3 target, double distancePercentage){
        if(isStarted){
            double magnitude = (new Vector2(velocity.getA(), velocity.getB())).distanceTo(Vector2.ZERO()) / (new Vector2(rawVelocity.getA(), rawVelocity.getB()).distanceTo(Vector2.ZERO()));
            if(magnitude > distancePercentage){
                return target.scale(kp);
            }else{
                return target.scale(distancePercentage);
            }
        }else{
            return target;
        }
    }
}
