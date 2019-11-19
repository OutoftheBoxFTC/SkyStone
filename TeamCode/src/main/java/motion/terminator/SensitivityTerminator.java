package motion.terminator;

public class SensitivityTerminator {
    private double range;
    private long time, lastTime;
    private boolean inclusive;
    public SensitivityTerminator(double time, double range, boolean inclusive){
        this.time = (long)(time*1.0e9);
        this.range = range;
        this.inclusive = inclusive;
    }

    public boolean shouldTerminate(double value, long now) {
        boolean conditionMet = inclusive?(Math.abs(value)<=range):(Math.abs(value)<range);
        if(time==0){
            return conditionMet;
        }
        if(conditionMet){
            if(lastTime==0){
                lastTime = now;
            } else if(now - lastTime >= time){
                lastTime = 0;
                return true;
            }
        } else {
            lastTime = 0;
        }
        return false;
    }
}
