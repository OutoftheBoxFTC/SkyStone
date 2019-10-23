package hardware.Sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(xmlTag="PixyCam2", name="CMULabs PixyCam v2", description="PixyCam vision sensor from CMULabs")
public class Pixycam extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public Pixycam(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        this.setOptimalReadWindow();
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
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
        FIRST(0),
        SYNC(0x00),
        SYNCCOMPLIMENT(0x01),
        CHECKSUM(0x02),
        CHECKSUMCOMPLIMENT(0x03),
        SIGNUM(0x04),
        SIGNUMCOMPLIMENT(0x05),
        XCENTER(0x06),
        XCENTERCOMPLIMENT(0x07),
        YCENTER(0x08),
        YCENTERCOMPLIMENT(0x09),
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
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(Register.FIRST.val, Register.LAST.val - Register.FIRST.val + 1, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value){
        deviceClient.write(reg.val, TypeConversion.shortToByteArray(value, ByteOrder.LITTLE_ENDIAN));
    }

    protected short readShort(Register reg){
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.val, 2), ByteOrder.LITTLE_ENDIAN);
    }

    public short getX(){
        return readShort(Register.XCENTER);
    }

    public short getY(){
        return readShort(Register.YCENTER);
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
}
