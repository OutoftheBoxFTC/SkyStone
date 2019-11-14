package hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Map;

import debug.FPSDebug;
import debug.SmartTelemetry;

public class Hardware {
    private LinearOpMode opMode;

    private double[] drivePowers, intakePowers, intakeServos;

    private SmartMotor a, b, c, d, left, right;
    private ExpansionHubServo leftServo, rightServo;
    private ExpansionHubEx hub1, hub2;
    private OpenCvCamera wc1;
    private OpenCvPipeline pipeline;

    private ArrayList<SmartMotor> driveMotors;

    private FPSDebug fpsDebug;

    private BNO055IMU imu;

    private SmartTelemetry telemetry;

    private ArrayList<HardwareDevice> registeredDevices, enabledDevices;

    private CalibrationData calibration;

    public Hardware(LinearOpMode opmode, SmartTelemetry telemetry){
        this.opMode = opmode;
        driveMotors = new ArrayList<>();
        drivePowers = new double[4];
        intakePowers = new double[2];
        registeredDevices = new ArrayList<>();
        enabledDevices = new ArrayList<>();
        fpsDebug = new FPSDebug(telemetry, "Hardware");
        this.telemetry = telemetry;
    }

    public void init(){
        HardwareMap map = opMode.hardwareMap;
        calibration = new CalibrationData();
        if(registeredDevices.contains(HardwareDevice.HUB_1_BULK)) {
            hub1 = getOrNull(map, ExpansionHubEx.class, "hub1");
        }
        if(registeredDevices.contains(HardwareDevice.HUB_2_BULK)){
            hub2 = getOrNull(map, ExpansionHubEx.class, "hub2");
        }
        if(registeredDevices.contains(HardwareDevice.WEBCAM_1)){
            int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
            wc1 = new OpenCvWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            wc1.openCameraDevice();
        }
        if(registeredDevices.contains(HardwareDevice.DRIVE_MOTORS)) {
            a = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "a"));
            b = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "b"));
            c = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "c"));
            d = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "d"));

            b.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            d.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            driveMotors.add(a);
            driveMotors.add(b);
            driveMotors.add(c);
            driveMotors.add(d);
        }
        if(registeredDevices.contains(HardwareDevice.INTAKE_MOTORS)){
            left = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "left"));
            right = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "right"));

        }
        if(registeredDevices.contains(HardwareDevice.INTAKE_SERVOS)){
            leftServo = (ExpansionHubServo) getOrNull(map.servo, "leftServo");
            rightServo = (ExpansionHubServo) getOrNull(map.servo, "rightServo");
        }
        if(registeredDevices.contains(HardwareDevice.GYRO)) {
            imu = getOrNull(map, BNO055IMU.class, "imu");
            if(imu != null) {
                initIMU();
            }
        }
    }

    public void calibrate(){
        //calibrates all analog devices
        if(registeredDevices.contains(HardwareDevice.HUB_1_BULK)) {
            calibration.addHub1BulkData(hub1.getBulkInputData());
        }
        if(registeredDevices.contains(HardwareDevice.HUB_2_BULK)){
            calibration.addHub2BulkData(hub2.getBulkInputData());
        }
        if(registeredDevices.contains(HardwareDevice.WEBCAM_1)){
            if(pipeline != null){
                wc1.setPipeline(pipeline);
            }
            wc1.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        }
        if(registeredDevices.contains(HardwareDevice.GYRO)){
            calibration.addGyroData(imu);
        }
    }

    private void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void drive(double a, double b, double c, double d){
        drivePowers = new double[]{a, b, c, d};
    }

    public void intake(double l, double r){
        intakePowers = new double[]{l, r};
    }

    public void intakeServos(double l, double r){
        intakeServos = new double[]{l, r};
    }

    public ReadData update() {
        fpsDebug.startIncrement();
        if(enabledDevices.contains(HardwareDevice.DRIVE_MOTORS)) {
            double[] drivePowers = this.drivePowers;
            if (drivePowers != null) {
                for (int i = 0; i < 4; i++) {
                    driveMotors.get(i).setPower(drivePowers[i]);
                }
            }
        }
        if(enabledDevices.contains(HardwareDevice.INTAKE_MOTORS)){
            if(intakePowers != null){
                left.setPower(intakePowers[0]);
                right.setPower(intakePowers[1]);
            }
        }
        if(enabledDevices.contains(HardwareDevice.INTAKE_SERVOS)){
            if(intakeServos != null){
                leftServo.setPosition(intakeServos[0]);
                rightServo.setPosition(intakeServos[1]);
            }
        }

        ReadData data = new ReadData(calibration);
        if(enabledDevices.contains(HardwareDevice.HUB_1_BULK)) {
            RevBulkData rawData = hub1.getBulkInputData();
            data.addHub1BulkData(rawData);
        }
        if(enabledDevices.contains(HardwareDevice.HUB_2_BULK)){
            RevBulkData rawData = hub2.getBulkInputData();
            data.addHub2BulkData(rawData);
        }
        if(enabledDevices.contains(HardwareDevice.GYRO)){
            data.addGyro(imu);
        }

        fpsDebug.endIncrement();
        fpsDebug.update();
        return data;
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
    }

    /**
     * Get the value associated with an id and instead of raising an error return null and log it
     *
     * @param map  the hardware map from the HardwareMap
     * @param name The ID in the hardware map
     * @param <T>  the type of hardware map
     * @return the hardware device associated with the name
     */
    public  <T extends com.qualcomm.robotcore.hardware.HardwareDevice> T getOrNull(HardwareMap.DeviceMapping<T> map, String name) {
        for (Map.Entry<String, T> item : map.entrySet()) {
            if (!item.getKey().equalsIgnoreCase(name)) {
                continue;
            }
            telemetry.setHeader(name, "found");
            return item.getValue();
        }
        telemetry.setHeader(name, "not found");
        RobotLog.e("ERROR: " + name + " not found!");
        return null;
    }

    public <T> T getOrNull(HardwareMap map, Class<T> type, String name) {
        try {
            T device = map.get(type, name);
            telemetry.setHeader(name, "found");
            return device;
        }
        catch (IllegalArgumentException e){
            telemetry.setHeader(name, "not found");
            RobotLog.e("ERROR: " + name + " not found!");
        }
        return null;
    }

    public BNO055IMU getIMU() {
        return imu;
    }

    public Hardware registerDevice(HardwareDevice device){
        registeredDevices.add(device);
        enabledDevices.add(device);
        return this;
    }

    public Hardware enableDevice(HardwareDevice device){
        if(registeredDevices.contains(device)) {
            enabledDevices.add(device);
        }
        return this;
    }

    public Hardware disableDevice(HardwareDevice device){
        if(enabledDevices.contains(device)) {
            enabledDevices.remove(device);
        }
        return this;
    }

    public void stop(){
        wc1.stopStreaming();
        wc1.closeCameraDevice();
    }

    public enum HardwareDevice {
        DRIVE_MOTORS,
        INTAKE_MOTORS,
        INTAKE_SERVOS,
        HUB_1_BULK,
        HUB_2_BULK,
        WEBCAM_1,
        GYRO
    }
}