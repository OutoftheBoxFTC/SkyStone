package debug;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

import debug.JsonFormat.Coordinates;
import debug.JsonFormat.RootObject;
import debug.JsonFormat.SensorIO;
import debug.JsonFormat.TelemetryData;
import math.Vector2;
import math.Vector3;

public class Connector {
    private static Connector instance = new Connector();
    private DatagramSocket socket;
    private Vector3 orientation = Vector3.ZERO();
    private List<String> telemetry, telemetryHeaders, sensorIO;
    private RootObject rootObject = new RootObject(new Coordinates(), new SensorIO(), new TelemetryData());
    private OutputStream stream;
    private DatagramPacket packet;
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
    }

    public void addTelemetry(String header, String data){
        if(telemetryHeaders.contains(header)){
            int index = telemetryHeaders.indexOf(header);
            telemetry.remove(index);
            telemetryHeaders.remove(index);
            telemetryHeaders.add(header);
            telemetry.add(header + ": " + data);
        }else{
            telemetryHeaders.add(header);
            telemetry.add(header + ": " + data);
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

    public void end() throws IOException {
        stream.flush();
        stream.close();
        socket.close();
    }
}
