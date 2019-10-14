package hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import revextensions2.RevBulkData;

public class CalibrationData {

    private int leftOffset, auxOffset, rightOffset;
    private double gyroOffset;

    public void addHub1BulkData(RevBulkData data){

    }
    public void addHub2BulkData(RevBulkData data){
        leftOffset = data.getMotorCurrentPosition(ReadData.LEFT);
        rightOffset = -data.getMotorCurrentPosition(ReadData.RIGHT);
        auxOffset = -data.getMotorCurrentPosition(ReadData.AUX);
    }

    public void addGyroData(BNO055IMU gyro){
        if(gyro != null) {
            Orientation orientation = gyro.getAngularOrientation();
            double yaw = orientation.firstAngle;
            double tau = Math.PI * 2;
            this.gyroOffset = ((yaw % tau) + tau) % tau;
        }
    }

    public int getLeftOffset() {
        return leftOffset;
    }

    public int getAuxOffset() {
        return auxOffset;
    }

    public int getRightOffset() {
        return rightOffset;
    }

    public double getGyroOffset() {
        return gyroOffset;
    }
}
