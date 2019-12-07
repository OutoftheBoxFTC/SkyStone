package Hardware.Sensors;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

import math.Vector3;

/***
 * Bytes    16-bit word    Description
 * ----------------------------------------------------------------
 * 0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
 * 2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
 * 4, 5     y              signature number
 * 6, 7     y              x center of object
 * 8, 9     y              y center of object
 * 10, 11   y              width of object
 * 12, 13   y              height of object
 */
//140, 73
@I2cDeviceType
@DeviceProperties(xmlTag="PixyCam2", name="CMULabs PixyCam v2", description="PixyCam vision sensor from CMULabs")
public class Pixycam extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    byte[] request = {(byte)0xae, (byte)0xc1, (byte)32, (byte)2, (byte)1, (byte)0xFF};
    byte[] pointRequest = {(byte)0xae, (byte)0xc1, (byte)112, (byte)5, (byte)0x9D, (byte)0x00, (byte)0x67, (byte)0x00, (byte)0};
    List<Short> shorts, prevShorts;
    public Pixycam(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        this.setOptimalReadWindow();
        super.registerArmingStateCallback(false);
        this.deviceClient.write(0, pointRequest);
        this.deviceClient.setHeartbeatAction(new I2cDeviceSynch.HeartbeatAction(false, true, null));
        this.deviceClient.setHeartbeatInterval(500);
        this.deviceClient.engage();
        prevShorts = new ArrayList<Short>();
        for(int i = 0; i < 8; i ++){
            prevShorts.add((short)-1);
        }
        shortToEndian((short)135);
        shortToEndian((short)139);
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    public enum Register{
        FIRST(0x51),
        SYNC(0x00),
        SYNCCOMPLIMENT(0x01),
        CHECKSUM(0x02),
        CHECKSUMCOMPLIMENT(0x03),
        SIGNUM(0x04),
        SIGNUMCOMPLIMENT(0x05),
        XCENTER(0x08),
        XCENTERCOMPLIMENT(0x09),
        YCENTER(0x0A),
        YCENTERCOMPLIMENT(0x0B),
        OBJWIDTH(0x0A),
        OBJWIDTHCOMPLIMENT(0x0B),
        OBJHEIGHT(0x0C),
        OBJHEIGHTCOMPLIMENT(0x0D),
        LAST(OBJHEIGHTCOMPLIMENT.val);
        public int val;

        Register(int val){
            this.val = val;
        }
    }

    protected void setOptimalReadWindow(){
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(Register.FIRST.val, 26, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value){
        deviceClient.write(reg.val, TypeConversion.shortToByteArray(value, ByteOrder.LITTLE_ENDIAN));
    }

    protected short readShort(Register reg){
        this.deviceClient.write(0, request);
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.val, 2), ByteOrder.LITTLE_ENDIAN);
    }

    public byte[] getCoordinateColor(){
        this.deviceClient.write(0, pointRequest);
        byte[] data = this.deviceClient.read(0, 9);
        //RobotLog.ii("data", getString(data));
        return data;
    }

    public int read8(int ireg){
        return (int) deviceClient.read8(ireg);
    }

    public short getX(){
        return shorts.get(5);
    }

    public short getY(){
        return shorts.get(6);
    }

    public short getWidth(){
        return readShort(Register.OBJWIDTH);
    }

    public short getHeight(){
        return readShort(Register.OBJHEIGHT);
    }

    public short getSigNum(){
        return readShort(Register.SIGNUM);
    }

    public List<Short> getShorts(){
        return shorts;
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

    private byte[] shortToEndian(Short s){
        ByteBuffer bb = ByteBuffer.allocate(2);
        bb.order(ByteOrder.LITTLE_ENDIAN);
        bb.putShort(s);
        bb.flip();
        byte[] buffer = new byte[2];
        buffer[0] = bb.get();
        buffer[1] = bb.get();
        RobotLog.i(buffer[0] + ", " + buffer[1]);
        return buffer;
    }

    public List<Short> getPrevShorts(){
        return prevShorts;
    }

    private String getString(byte[] b){
        String s = "";
        for(byte bb : b){
            s += ((int)bb) + ",";
        }
        return s;
    }
}
