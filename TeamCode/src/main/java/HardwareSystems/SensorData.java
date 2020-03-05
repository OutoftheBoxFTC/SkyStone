package HardwareSystems;

public class SensorData {
    private double leftEncoder, rightEncoder, auxEncoder, gyro, lift, intakeTripwire, leftLaser, rightLaser, rawLift;
    private byte[] pixy;
    private boolean liftLimit;
    private long timestamp;
    CalibrationSystem calibration;
    public SensorData(CalibrationSystem calibration, long timestamp){
        this.timestamp = timestamp;
        this.calibration = calibration;
        this.leftEncoder = 0;
        this.rightEncoder = 0;
        this.auxEncoder = 0;
        this.gyro = 0;
        this.lift = 0;
        this.intakeTripwire = 0;
        this.leftLaser = 0;
        this.rightLaser = 0;
        this.rawLift = 0;
    }
    public void setTimestamp(long timestamp){
        this.timestamp = timestamp;
    }
    public void setCalibration(CalibrationSystem calibration){
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
        this.rawLift = position;
    }

    public double getRawLift() {
        return rawLift;
    }

    public double getLift(){
        return lift;
    }

    public void setIntakeTripwire(double value){
        intakeTripwire = value;
    }

    public double getIntakeTripwire(){
        return intakeTripwire;
    }

    public void setLiftLimit(boolean liftLimit){
        this.liftLimit = liftLimit;
    }
    public boolean getLiftLimit(){
        return liftLimit;
    }

    public void setRightLaser(double rightLaser) {
        this.rightLaser = rightLaser;
    }

    public void setLeftLaser(double leftLaser) {
        this.leftLaser = leftLaser;
    }

    public double getLeftLaser() {
        return leftLaser;
    }

    public double getRightLaser() {
        return rightLaser;
    }

    public CalibrationSystem getCalibration(){
        return calibration;
    }

    public long getTimestamp(){
        return timestamp;
    }
}
