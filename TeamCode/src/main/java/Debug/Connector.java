package Debug;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.*;
import java.net.*;
import java.util.*;

import Debug.JsonFormat.*;
import math.*;

public class Connector {
    private static Connector instance = new Connector();
    private DatagramSocket socket;
    private Vector3 orientation = Vector3.ZERO();
    private List<String> telemetry, telemetryHeaders, sensorIO;
    private RootObject rootObject = new RootObject(new Coordinates(), new SensorIO(), new TelemetryData());
    private OutputStream stream;
    private DatagramPacket packet;
    private Receiver reciever;
    private Connector(){ }

    public static Connector getInstance(){
        return instance;
    }

    public void start() throws IOException {
        telemetry = new ArrayList<>();
        telemetryHeaders = new ArrayList<>();
        sensorIO = new ArrayList<>();
        socket = new DatagramSocket(1119);
        socket.setReuseAddress(true);
        socket.setBroadcast(true);
        reciever = new Receiver();
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

    public void addSensorIO(String data){
        sensorIO.add(data);
    }

    public void addOrientation(Vector2 position, double rotation){
        orientation = new Vector3(position.getA(), position.getB(), rotation);
    }

    public void addOrientation(Vector3 position){
        orientation = position;
    }

    public void update() throws IOException {
        rootObject.TelemetryData.Data = telemetry;
        rootObject.SensorIO.data = sensorIO;
        rootObject.Coordinates.x = orientation.getA();
        rootObject.Coordinates.y = orientation.getB();
        rootObject.Coordinates.rot = orientation.getC();
        String json = SimpleGson.getInstance().toJson(rootObject);
        byte[] toSend = json.getBytes();
        int sendLen = toSend.length;
        packet = new DatagramPacket(toSend, sendLen, InetAddress.getByName("255.255.255.255"), 1119);
        socket.send(packet);
        for(int i = 0; i < telemetry.size(); i ++){
            telemetry.remove(i);
            telemetryHeaders.remove(i);
        }
    }

    public RecievePacket getDataFromMaster(long timeout) {
        timeout = timeout + System.currentTimeMillis();
        while(timeout > System.currentTimeMillis() && !reciever.run());
        RecievePacket format = SimpleGson.getInstance().fromJson(reciever.getPacket(), RecievePacket.class);
        return format;
    }

    public void end() throws IOException {
        stream.flush();
        stream.close();
        socket.close();
    }
}
