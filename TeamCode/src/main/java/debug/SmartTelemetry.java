package debug;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import math.Vector3;

public class SmartTelemetry {
    private Telemetry telemetry;
    private ArrayList<String[]> pingMessages;
    private ArrayList<Long> endTimes;
    private Connector connector;
    private Map<String, Object> headerMessages;
    private boolean loggerEnabled, loggerStarted;
    private double time;

    public SmartTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        headerMessages = new LinkedHashMap<>();
        pingMessages = new ArrayList<>();
        endTimes = new ArrayList<>();
        connector = Connector.getInstance();
        loggerEnabled = false;
        loggerStarted = false;
        time = 0;
    }
    @Deprecated
    public void pingMessage(String header, String message, long timeMs){
        pingMessages.add(new String[]{header, message});
        endTimes.add(System.currentTimeMillis()+timeMs);
    }

    public void update(){
        for (int i = 0; i < pingMessages.size();) {
            if(System.currentTimeMillis() > endTimes.get(i)){
                endTimes.remove(i);
                pingMessages.remove(i);
            } else {
                if (pingMessages.get(i).length == 2) {
                    String[] message = pingMessages.get(i);
                    telemetry.addData(message[0], message[1]);
                }
            }
        }

        for(String header : headerMessages.keySet()){
            telemetry.addData(header, headerMessages.get(header));
            if(loggerEnabled) {
                if (header.toLowerCase().equals("position")) {
                    connector.addOrientation((Vector3) headerMessages.get(header));
                } else {
                    connector.addTelemetry(header, headerMessages.get(header).toString());
                }
            }
        }
        telemetry.update();
        long now = System.currentTimeMillis();
        if(loggerEnabled && now - time > 50) {
            try {
                connector.update();
            } catch (IOException e) {
                e.printStackTrace();
            }
            time = now;
        }
    }

    public void setHeader(String header, Object message){
        headerMessages.put(header, message);
    }

    public void clearHeaders(String... headers){
        for(String header : headers) {
            if(headerMessages.keySet().contains(header)) {
                headerMessages.remove(header);
            }
        }
    }

    public void clearAllHeadersExcept(String... keep){
        Set<String> headers = headerMessages.keySet();
        ArrayList<String> keepList = new ArrayList<>(Arrays.asList(keep));
        ArrayList<String> removed = new ArrayList<>();
        for (String header : headers) {
            if(!keepList.contains(header)){
               removed.add(header);
            }
        }

        for(String header : removed){
            headerMessages.remove(header);
        }
    }

    public void stop() throws IOException {
        connector.end();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void enableLogger(){
        loggerEnabled = true;
        if(!loggerStarted){
            loggerStarted = true;
            try {
                connector.start();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
