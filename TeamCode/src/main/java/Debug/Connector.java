package Debug;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import Debug.EditableValue.EditableValue;
import math.Vector2;
import math.Vector3;

public class Connector {
    private static Connector instance = new Connector();
    private DatagramSocket socket;
    private Vector3 orientation = Vector3.ZERO();
    private List<String> telemetry, telemetryHeaders, sensorIO;
    private List<EditableValue> editableVariables;
    private ArrayList<Double> graphVals;
    private OutputStream stream;
    private DatagramPacket packet;
    private Receiver reciever;
    private long timer = 0;
    private Connector(){ }

    public static Connector getInstance(){
        return instance;
    }

    public void start() throws IOException {
        telemetry = new ArrayList<>();
        telemetryHeaders = new ArrayList<>();
        sensorIO = new ArrayList<>();
        editableVariables = new ArrayList<>();
        socket = new DatagramSocket(1119);
        socket.setReuseAddress(true);
        socket.setBroadcast(true);
        reciever = new Receiver();
        graphVals = new ArrayList<>();
        graphVals.add(10.0);
    }

    public void addTelemetry(String header, Object data){
        if(telemetryHeaders.contains(header)){
            int index = telemetryHeaders.indexOf(header);
            telemetry.remove(index);
            telemetryHeaders.remove(index);
            telemetryHeaders.add(header);
            telemetry.add(header + ": " + data.toString());
        }else{
            telemetryHeaders.add(header);
            telemetry.add(header + ": " + data.toString());
        }
    }

    public void addSensorIO(String sensor, String data){

        sensorIO.add(sensor + ": " + data);
    }

    public void addGraphVal(int x, double y){
        if(x < graphVals.size()) {
            graphVals.add(x, y);
        }else{
            graphVals.add(y);
        }
    }

    public void addOrientation(Vector2 position, double rotation){
        orientation = new Vector3(position.getA(), position.getB(), rotation);
    }

    public void addOrientation(Vector3 position){
        orientation = position;
    }

    public void addEditableVariabe(EditableValue editableValue){
        if(!editableVariables.contains(editableValue)) {
            editableVariables.add(editableValue);
        }
    }

    public void update() throws IOException, JSONException {
        JSONObject jsonObject = new JSONObject();
        JSONArray telemetryJson = new JSONArray();
        for(String s : telemetry){
            telemetryJson.put(s);
        }
        JSONArray sensorIOJson = new JSONArray();
        for(String s : sensorIO){
            sensorIOJson.put(s);
        }
        jsonObject.put("telemetry", telemetryJson);
        jsonObject.put("sensorIO", sensorIOJson);
        jsonObject.put("orientation", orientation.toString());
        String json = jsonObject.toString();
        byte[] toSend = json.getBytes();
        int sendLen = toSend.length;
        packet = new DatagramPacket(toSend, sendLen, InetAddress.getByName("255.255.255.255"), 1119);
        if(System.currentTimeMillis() > timer) {
            socket.send(packet);
            timer = System.currentTimeMillis() + 50;
        }
        telemetry.clear();
        telemetryHeaders.clear();
        sensorIO.clear();
    }

    public RecievePacket getDataFromMaster(long timeout) {
        timeout = timeout + System.currentTimeMillis();
        while(timeout > System.currentTimeMillis() && !reciever.run());
        String s = reciever.getPacket();
        if(!s.equals("No Data")) {
            return SimpleGson.getInstance().fromJson(s, RecievePacket.class);
        }
        return null;
    }

    public void end() throws IOException {
        stream.flush();
        stream.close();
        socket.close();
    }
}
