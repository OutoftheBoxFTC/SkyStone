package Hardware;

import math.Vector3;

public class SensorData {
    private double leftEncoder, rightEncoder, auxEncoder, gyro, lift;
    private byte[] pixy;
    private long timestamp;
    CalibrationSystem calibration;
    public SensorData(CalibrationSystem calibration, long timestamp){
        this.timestamp = timestamp;
        this.calibration = calibration;
    }
    public void setOdometryEncoders(double leftEncoder, double rightEncoder, double auxEncoder){
        this.leftEncoder = leftEncoder - calibration.getLeft();
        this.rightEncoder = rightEncoder - calibration.getRight();
        this.auxEncoder = auxEncoder - calibration.getAux();
    }
    public void setGyro(double gyro){
        this.gyro = gyro - calibration.getGyro();
    }
    public double getLeft(){
        return leftEncoder;
    }
    public void setPixy(byte[] pixy){
        this.pixy = pixy;
    }
    public double getRight(){
        return rightEncoder;
    }
    public double getAux(){
        return auxEncoder;
    }
    public double getGyro(){
        return gyro;
    }
    public byte[] getPixy(){
        return pixy;
    }
    public void setLift(double position){
        this.lift = position - calibration.getLift();
    }
    public double getLift(){
        return lift;
    }
    public long getTimestamp(){
        return timestamp;
    }
}
