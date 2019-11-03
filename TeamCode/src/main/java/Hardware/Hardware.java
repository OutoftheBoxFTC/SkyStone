package Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

import Hardware.HardwareDevices.SmartMotor;
import Hardware.HardwareDevices.SmartServo;
import math.Vector2;
import math.Vector4;

public class Hardware {
    private SmartMotor frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight;
    private DcMotor odometryRightMotor, odometryLeftMotor;
    private SmartServo leftLatch, rightLatch;
    private OpMode opmode;
    private Telemetry telemetry;
    private ArrayList<HardwareDevices> enabledDevices;
    private CalibrationSystem calibration;
    private BNO055IMU imu;
    public Hardware(OpMode opmode, Telemetry telemetry){
        this.opmode = opmode;
        this.telemetry = telemetry;
        enabledDevices = new ArrayList<>();
    }
    public void init(){
        HardwareMap map = opmode.hardwareMap;
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)){
            frontLeft = new SmartMotor(getOrNull(map, DcMotor.class, "fl"));
            frontRight = new SmartMotor(getOrNull(map, DcMotor.class, "fr"));
            backLeft = new SmartMotor(getOrNull(map, DcMotor.class, "bl"));
            backRight = new SmartMotor(getOrNull(map, DcMotor.class, "br"));
            odometryRightMotor = getOrNull(map, DcMotor.class, "odometryR");
            odometryLeftMotor = getOrNull(map, DcMotor.class, "odometryL");
        }
        if(enabledDevices.contains(HardwareDevices.LATCH_SERVOS)){
            leftLatch = new SmartServo(getOrNull(map, Servo.class, "leftLatch"));
            rightLatch = new SmartServo(getOrNull(map, Servo.class, "rightLatch"));
        }
        if(enabledDevices.contains(HardwareDevices.INTAKE_MOTORS)){
            intakeLeft = new SmartMotor(getOrNull(map, DcMotor.class, "intakeLeft"));
            intakeRight = new SmartMotor(getOrNull(map, DcMotor.class, "intakeRight"));
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)){
            imu = getOrNull(map, BNO055IMU.class, "imu");
        }
    }

    public void calibrate(){
        calibration = new CalibrationSystem();
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)) {
            calibration.setOdometryEncoders(odometryRightMotor.getCurrentPosition(), odometryLeftMotor.getCurrentPosition(), backRight.getMotor().getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)) {
            Orientation orientation = imu.getAngularOrientation();
            double yaw = orientation.firstAngle;
            double tau = Math.PI * 2;
            calibration.setGyro(((yaw % tau) + tau) % tau);
        }
    }

    public SensorData update(HardwareData data){
        SensorData sensors = new SensorData(calibration, System.currentTimeMillis());
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)){
            Vector4 motorPowers = data.getMotorPowers();
            frontLeft.setPower(motorPowers.getA());
            frontRight.setPower(motorPowers.getB());
            backLeft.setPower(motorPowers.getC());
            backRight.setPower(motorPowers.getD());
        }
        if(enabledDevices.contains(HardwareDevices.LATCH_SERVOS)){
            Vector2 servoPositions = data.getLatchPositions();
            leftLatch.setPosition(servoPositions.getA());
            rightLatch.setPosition(servoPositions.getB());
        }
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)) {
            sensors.setOdometryEncoders(odometryRightMotor.getCurrentPosition(), odometryLeftMotor.getCurrentPosition(), backRight.getMotor().getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)) {
            Orientation orientation = imu.getAngularOrientation();
            double yaw = orientation.firstAngle;
            double tau = Math.PI * 2;
            sensors.setGyro(((yaw % tau) + tau) % tau);
        }
        return sensors;
    }

    public <T> T getOrNull(HardwareMap map, Class<T> type, String name) {
        try {
            T device = map.get(type, name);
            telemetry.addData(name, "found");
            return device;
        }
        catch (IllegalArgumentException e){
            telemetry.addData(name, "not found");
            RobotLog.e("ERROR: " + name + " not found!");
        }
        return null;
    }

    public void enableAll(){
        enabledDevices.add(HardwareDevices.DRIVE_MOTORS);
        enabledDevices.add(HardwareDevices.LATCH_SERVOS);
        enabledDevices.add(HardwareDevices.INTAKE_MOTORS);
        enabledDevices.add(HardwareDevices.GYRO);
    }
    public void enableDevice(HardwareDevices device){
        if(!enabledDevices.contains(device)){
            enabledDevices.add(device);
        }
    }
    public void disableDevice(HardwareDevices device){
        if(enabledDevices.contains(device)){
            enabledDevices.remove(device);
        }
    }
    public enum HardwareDevices{
        DRIVE_MOTORS,
        LATCH_SERVOS,
        INTAKE_MOTORS,
        GYRO,
    }
}
