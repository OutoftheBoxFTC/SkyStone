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

    public PIDControl(double kp, double ki, double kd, boolean integralReset){
        this(kp, ki, kd);
        this.integralReset = integralReset;
    }

    public double evaluation(double error, long timeStamp){
        if(lastTime==0){
            previousError = error;
            lastTime = timeStamp;
            return error*kp;
        }
        double dt = (timeStamp-lastTime)/1.0e9;
        lastTime = timeStamp;
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
