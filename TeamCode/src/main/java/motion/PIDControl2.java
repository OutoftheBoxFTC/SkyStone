package motion;

import math.Vector2;

public class PIDControl2 {
    private final double kp, ki, kd;

    private Vector2 integral, previousError;
    private boolean integralReset;
    private long lastTime;

    public PIDControl2(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public Vector2 evaluation(Vector2 error){
        long now = System.nanoTime();
        if(lastTime==0){
            previousError = error;
            lastTime = now;
            return new Vector2(0, 0);
        }
        double dt = (now-lastTime)/1.0e9;
        lastTime = now;
        if(integralReset) {
            if (error.getA() * previousError.getA() < 0){
                integral.setA(0);
            }
            if(error.getB()*previousError.getB()<0){
                integral.setB(0);
            }
        }
        integral.add(error.scale(dt));
        Vector2 derivative = error.add(previousError.scale(-1)).scale(1/dt);
        return error.scale(kp).add(integral.scale(ki)).add(derivative.scale(kd));
    }

    public void reset(){
        integral = new Vector2(0, 0);
        previousError = null;
        lastTime = 0;
    }

    public double getKp() {
        return kp;
    }
}
