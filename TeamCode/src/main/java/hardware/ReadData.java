package hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.RevBulkData;

public class ReadData {
    public static final int LEFT = 1, AUX = 2, RIGHT = 0;
    private int left, right, aux, vLeft, vRight, vAux;
    private double gyro;
    private CalibrationData calibration;
    private long hub1BulkTime, hub2BulkTime, gyroTime;

    public ReadData(CalibrationData calibration){
        this.calibration = calibration;
    }

    public int getLeft() {
        return left;
    }

    public int getRight() {
        return right;
    }

    public int getAux() {
        return aux;
    }

    public int getvLeft() {
        return vLeft;
    }

    public int getvRight() {
        return vRight;
    }

    public int getvAux() {
        return vAux;
    }

    public double getGyro() {
        return gyro;
    }

    public long getGyroTime() {
        return gyroTime;
    }

    public long getHub1BulkTime() {
        return hub1BulkTime;
    }

    public void addGyro(BNO055IMU gyro) {
        gyroTime = System.nanoTime();
        if(gyro != null) {
            Orientation orientation = gyro.getAngularOrientation();
            double yaw = orientation.firstAngle;
            double tau = Math.PI * 2;
            if (calibration != null) {
                yaw -= calibration.getGyroOffset();
            }
            this.gyro = ((yaw % tau) + tau) % tau;
        }
    }

    public void addHub1BulkData(RevBulkData data){
        hub1BulkTime = System.nanoTime();
    }

    public void addHub2BulkData(RevBulkData data){
        this.hub2BulkTime = System.nanoTime();
        left = data.getMotorCurrentPosition(LEFT);
        right = -data.getMotorCurrentPosition(RIGHT);
        aux = -data.getMotorCurrentPosition(AUX);
        if(calibration != null){
            left -= calibration.getLeftOffset();
            right -= calibration.getRightOffset();
            aux -= calibration.getAuxOffset();
        }
        vLeft = data.getMotorVelocity(LEFT);
        vRight = data.getMotorVelocity(RIGHT);
        vAux = data.getMotorVelocity(AUX);
    }
}