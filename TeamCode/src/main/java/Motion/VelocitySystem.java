package Motion;

import HardwareSystems.SensorData;
import math.Vector2;
import math.Vector3;

public class VelocitySystem {
    Vector3 velocity, target, rawVelocity, tempVelocity;
    double MAX_VELOCITY = 100, MIN_SPEED = 0.1, kp = 0.01;
    boolean isStarted = false;
    public VelocitySystem(){
        target = Vector3.ZERO();
        velocity = Vector3.ZERO();
    }

    public void start(Vector3 velocity){
        this.rawVelocity = velocity;
        this.isStarted = true;
    }

    public void setTargetVelocity(Vector3 target){
        this.target = target;
    }

    public Vector3 update(SensorData sensors){
        if(isStarted) {
            Vector3 toReturn = Vector3.ZERO();
            tempVelocity = (rawVelocity.scale(1 / MAX_VELOCITY));
            velocity = new Vector3(new Vector2(velocity.getA(), velocity.getB()).distanceTo(Vector2.ZERO()) * Math.cos(Math.atan2(velocity.getB(), velocity.getA()) + sensors.getGyro()), new Vector2(velocity.getA(), velocity.getB()).distanceTo(Vector2.ZERO()) * Math.sin(Math.atan2(velocity.getB(), velocity.getA()) + sensors.getGyro()), tempVelocity.getC());
            if (target.getA() == 0) {
                toReturn.setA(0);
            } else {
                if ((target.getA() - velocity.getA()) * kp < 0.01) {
                    toReturn.add(((target.getA() - velocity.getA()) * kp) / Math.abs((target.getA() - velocity.getA()) * kp) * 0.01, 0, 0);
                } else {
                    toReturn.add((target.getA() - velocity.getA()) * kp, 0, 0);
                }
            }
            if (target.getB() == 0) {
                toReturn.setB(0);
            } else {
                if ((target.getB() - velocity.getB()) * kp < 0.01) {
                    toReturn.add(0, ((target.getB() - velocity.getB()) * kp) / Math.abs((target.getB() - velocity.getB()) * kp) * 0.01, 0);
                } else {
                    toReturn.add(0, (target.getB() - velocity.getB()) * kp, 0);
                }
            }
            toReturn.setC(target.getC());
            if (toReturn.getA() < MIN_SPEED) {
                toReturn.setA(MIN_SPEED * (toReturn.getA() / Math.abs(toReturn.getA())));
            }
            if (toReturn.getB() < MIN_SPEED) {
                toReturn.setB(MIN_SPEED * (toReturn.getB() / Math.abs(toReturn.getB())));
            }
            return toReturn;
        }else{
            return target;
        }
    }
}
