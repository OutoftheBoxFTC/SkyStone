package hardware;

import android.graphics.Rect;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.RevBulkData;

public class ReadData {
    public static final int LEFT = 2, AUX = 1, RIGHT = 2;
    private int left, right, aux, vLeft, vRight, vAux;
    private Rect leftPixyBound, rightPixyBound;
    private double gyro;
    private CalibrationData calibration;
    private long hub1BulkTime, hub2BulkTime, gyroTime, startTime, driveUpdateTime;

    public ReadData(CalibrationData calibration){
        this.calibration = calibration;
        leftPixyBound = new Rect();
        rightPixyBound = new Rect();
        startTime = System.nanoTime();
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

    public Rect getLeftPixyBound() {
        return leftPixyBound;
    }

    public Rect getRightPixyBound() {
        return rightPixyBound;
    }

    public long getGyroTime() {
        return gyroTime;
    }

    public long getHub1BulkTime() {
        return hub1BulkTime;
    }

    public long getStartTime() {
        return startTime;
    }

    public long getDriveUpdateTime() {
        return driveUpdateTime;
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
        this.hub1BulkTime = System.nanoTime();
        left = -data.getMotorCurrentPosition(LEFT);
        aux = data.getMotorCurrentPosition(AUX);
        if(calibration != null){
            left -= calibration.getLeftOffset();
            aux -= calibration.getAuxOffset();
        }
        vLeft = data.getMotorVelocity(LEFT);
        vAux = data.getMotorVelocity(AUX);
    }

    public void addHub2BulkData(RevBulkData data){
        this.hub2BulkTime = System.nanoTime();
        right = data.getMotorCurrentPosition(RIGHT);

        if(calibration != null){
            right -= calibration.getRightOffset();
        }
        vRight = data.getMotorVelocity(RIGHT);
    }

    public void addLeftPixy(Pixycam leftPixy){
        leftPixy.queueData();
        int x = leftPixy.getX(), y = leftPixy.getY();
        leftPixyBound.set(x-leftPixy.getX(), y-leftPixy.getY(), x+leftPixy.getX(), y+leftPixy.getY());
    }

    public void addRightPixy(Pixycam rightPixy){
        rightPixy.queueData();
        int x = rightPixy.getX(), y = rightPixy.getY();
        rightPixyBound.set(x-rightPixy.getX(), y-rightPixy.getY(), x+rightPixy.getX(), y+rightPixy.getY());
    }

    public void updateDrive(){
        driveUpdateTime = System.nanoTime();
    }
}