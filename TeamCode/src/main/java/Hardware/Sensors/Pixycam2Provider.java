package Hardware.Sensors;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
0        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
1        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
2        4, 5     y              signature number
3        6, 7     y              x center of object
4        8, 9     y              y center of object
5        10, 11   y              width of object
6        12, 13   y              height of object
        */
public class Pixycam2Provider {
    I2cDeviceSynch pixy;
    byte[] data = new byte[12];
    byte[] request = {(byte)0xae, (byte)0xc1, (byte)32, (byte)2, (byte)0xFF, (byte)0xFF};
    byte[] lampRequest = {(byte)0xae, (byte)0xc1, (byte)22, (byte)2, (byte)1, (byte)1};
    byte[] colorChange = {(byte)0xae, (byte)0xcl, (byte)20, (byte)3, (byte)0, (byte)0, (byte)0};
    public List<Short> pixyData;

    /**
     * Initializer, used to set the pixycam and which mineral it is detecting
     * @param pixy an I2cDeviceSync class representing the pixy cam
     */
    public Pixycam2Provider(I2cDeviceSynch pixy){
        this.pixy = pixy;
        request[4] = (byte) 1;
    }

    /**
     * Update funtion, queries the pixy cam with data header 0x51 if it is looking at gold minerals, or 0x52 if looking at silver minerals, than stores it to the Bye[] pixyData variable
     * During the update, a block with a header is sent to the pixy, and the pixy returns 5 bytes containing information about what it sees
     * */
    public void update(){
        pixy.write(0, request);
        data = pixy.read(0, 26);
        pixyData = endianToShort(data);
    }

    /**
     * Engages the pixy, turning it on and quering data for the first time. Must be run in init, to give enough time for it to turn on
     */
    public void start(){
        pixy.engage();
    }

    /**
     * These functions return data from the pixy cam, getting data about the largest object seen
     * @return returns the data, a rounded byte from 0-255
     */
    public int getX(){
        if(pixyData.get(4) > 0){
            return pixyData.get(4);
        }else{
            return -1;
        }
    }
    public int getY(){
        if(pixyData.get(5) > 0){
            return pixyData.get(5);
        }else{
            return -1;
        }
    }
    public int getWidth(){
        if(pixyData.get(6) > 0){
            return pixyData.get(6);
        }else{
            return -1;
        }
    }
    public int getHeight(){
        if(pixyData.get(7) > 0){
            return pixyData.get(7);
        }else{
            return -1;
        }
    }

    /**
     * Returns if the pixy cam is currently on
     * @return boolean if the pixy is on
     */
    public boolean isEngaged(){
        return pixy.isEngaged();
    }

    public void setLamps(boolean headlights, boolean colorLight){
        if(headlights){
            lampRequest[4] = (byte)1;
        }else{
            lampRequest[4] = (byte)0;
        }
        if(colorLight){
            lampRequest[5] = (byte)1;
        }else{
            lampRequest[5] = (byte)0;
        }
        pixy.write(0, lampRequest);
    }

    public void setLED(int r, int g, int b){
        colorChange[4] = (byte)r;
        colorChange[5] = (byte)g;
        colorChange[6] = (byte)b;
        pixy.write(0, colorChange);
    }


    private List<Short> endianToShort(byte[] byteArray){
        List<Short> shorts = new ArrayList<Short>();
        ByteBuffer bb = ByteBuffer.wrap(byteArray);
        bb.order( ByteOrder.LITTLE_ENDIAN);
        while( bb.hasRemaining()) {
            shorts.add(bb.getShort());
        }
        return shorts;
    }
}