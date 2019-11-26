package Debug;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.*;
import java.net.*;
import java.util.*;

public class Receiver {
    DatagramSocket socket;
    List packetBuffer;

    /**
     * Creates a new Receiver bound to port 2026 (default)
     */
    public Receiver(){
        try {
            socket = new DatagramSocket(2026);
            socket.setReuseAddress(true);
            socket.setSoTimeout(100);
        } catch (SocketException e) {
            e.printStackTrace();
        }
        packetBuffer = Collections.synchronizedList(new ArrayList<DatagramPacket>());
    }

    /**
     * Runs the receiver. Stalls the thread until it receives new data. The system will timeout after 1 second with a java.net.SocketTimeoutException if no new data is received
     */
    public boolean run() {
        byte[] buffer = new byte[1024];
        try {
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, InetAddress.getByName("255.255.255.255"), 2026);
            socket.receive(packet);
            RobotLog.ii("Packet", new String(packet.getData()));
            packetBuffer.add(packet);
        }catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    /**
     * Gets the latest packet of data from the computer
     * @return the packet, in a String format
     */
    public String getPacket(){
        if(packetBuffer.size() > 0) {
            DatagramPacket toReturn = (DatagramPacket) packetBuffer.get(0);
            if (packetBuffer.size() > 1) {
                packetBuffer.remove(0);
            }
            String s = new String(toReturn.getData()).substring(0, new String(toReturn.getData()).indexOf('}') + 1);
            return s;
        }else{
            RobotLog.e("No Data");
            return "No Data";
        }
    }
}
