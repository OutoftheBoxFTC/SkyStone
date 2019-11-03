package Hardware;

/**
 * Calibration Data to calibrate everything
 */
public class CalibrationSystem {
    private double leftEncoder, rightEncoder, auxEncoder, gyro, pixyData;
    private long timestamp;

    public CalibrationSystem(){ }

    public void setOdometryEncoders(double leftEncoder, double rightEncoder, double auxEncoder){
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.auxEncoder = auxEncoder;
    }
    public void setGyro(double gyro){
        this.gyro = gyro;
    }
    public double getLeft(){
        return leftEncoder;
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
}
