package motion;

public class PIDControl {
    private final double kp, ki, kd;

    private double integral, previousError;
    private boolean integralReset;
    private long lastTime;

    public PIDControl(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double evaluation(double error){
        long now = System.nanoTime();
        if(lastTime==0){
            previousError = error;
            lastTime = now;
            return 0;
        }
        double dt = (now-lastTime)/1.0e9;
        lastTime = now;
        if(integralReset) {
            if (error * previousError < 0){
                integral = 0;
            }
        }
        integral += error*dt;
        double derivative = (error-previousError)/dt;
        return kp*error+ki*integral+kd*derivative;
    }

    public void reset(){
        integral = 0;
        previousError = 0;
        lastTime = 0;
    }
}
