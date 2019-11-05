package Hardware;

/**
 * Calibration Data to calibrate everything
 */
public class CalibrationSystem {
    private double leftEncoder, rightEncoder, auxEncoder, gyro, pixyData;
    private long timestamp;

    /**
     * Stores calibration data for sensors
     */
    public CalibrationSystem(){ }

    /**
     * Sets the calibration for the odometry encoders
     * @param leftEncoder the left encoder value
     * @param rightEncoder the right encoder value
     * @param auxEncoder the aux encoder value
     */
    public void setOdometryEncoders(double leftEncoder, double rightEncoder, double auxEncoder){
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.auxEncoder = auxEncoder;
    }

    /**
     * Sets the calibration for the gyro
     * @param gyro the gyro values
     */
    public void setGyro(double gyro){
        this.gyro = gyro;
    }

    /**
     * gets the left encoder calibration value
     * @return the left encoder calibration value
     */
    public double getLeft(){
        return leftEncoder;
    }

    /**
     * gets the right encoder calibration value
     * @return the right encoder calibration value
     */
    public double getRight(){
        return rightEncoder;
    }

    /**
     * gets the aux encoder calibration value
     * @return the aux encoder calibration value
     */
    public double getAux(){
        return auxEncoder;
    }

    /**
     * gets the gyro calibration value
     * @return the gyro calibration value
     */
    public double getGyro(){
        return gyro;
    }
}
