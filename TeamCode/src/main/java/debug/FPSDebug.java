package debug;

import java.util.ArrayList;

public class FPSDebug {
    private ArrayList<Long> timeIncrements, timeStamps;
    private SmartTelemetry telemetry;
    private long lastTime = 0;
    private String loopName;
    double fpsAvg, timeAvg;

    long totalTime;

    public FPSDebug(SmartTelemetry telemetry, String loopName){
        timeIncrements = new ArrayList<>();
        timeStamps = new ArrayList<>();
        this.telemetry = telemetry;
        this.loopName = loopName;
        totalTime = 0;
    }

    public void update() {
        long now = timeStamps.get(timeStamps.size()-1);
        while (now - timeStamps.get(0)>1e9){
            totalTime -= timeIncrements.remove(0);
            timeStamps.remove(0);
        }
        this.fpsAvg = Math.round((double)timeIncrements.size()/totalTime*1.0e12)/1.0e3;
        this.timeAvg = Math.round((double)totalTime/1.0e3/timeIncrements.size())/1.0e3;
    }

    public void queryFPS(){
        telemetry.setHeader(loopName + " FPS", String.valueOf((int) fpsAvg));
    }

    public void queryDuration(){
        telemetry.setHeader(loopName + " Duration", String.valueOf(timeAvg) + "ms");
    }

    public void startIncrement() {
        if(lastTime==0) {
            lastTime = System.nanoTime();
        }
    }

    public void endIncrement(){
        if(lastTime==0)
            return;
        long timeIncrement = System.nanoTime()-lastTime;
        totalTime += timeIncrement;
        timeIncrements.add(timeIncrement);
        timeStamps.add(lastTime);
        lastTime = 0;
    }
}