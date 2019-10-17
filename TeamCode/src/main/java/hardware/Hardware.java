package hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Map;

import debug.FPSDebug;
import debug.SmartTelemetry;

public class Hardware implements Runnable {
    private static final long MIN_WAIT_TIME=1000000;

    private LinearOpMode opMode;

    private ArrayList<ReadData> dataBuffer;
    private ArrayList<double[]> drivePowerBuffer;

    private SmartMotor a, b, c, d;
    private ExpansionHubEx hub1, hub2;

    private ArrayList<SmartMotor> driveMotors;

    private FPSDebug fpsDebug;

    private BNO055IMU imu;

    private boolean dataLogged;

    private SmartTelemetry telemetry;

    private ArrayList<HardwareDevice> registeredDevices, enabledDevices;

    private CalibrationData calibration;

    public Hardware(LinearOpMode opmode, SmartTelemetry telemetry){
        this.opMode = opmode;
        driveMotors = new ArrayList<>();
        dataBuffer = new ArrayList<>();
        drivePowerBuffer = new ArrayList<>();
        registeredDevices = new ArrayList<>();
        enabledDevices = new ArrayList<>();
        fpsDebug = new FPSDebug(telemetry, "Hardware");
        this.telemetry = telemetry;
        dataLogged = false;
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
        drivePowerBuffer.add(new double[]{a, b, c, d});
    }

    @Override
    public void run() {
        ArrayList<HardwareDevice> enabledDevices = new ArrayList<>();
        while (!opMode.isStopRequested()){
            long startTime = System.nanoTime();
            fpsDebug.startIncrement();
            boolean drivePowersBuffered = !drivePowerBuffer.isEmpty();
            if(enabledDevices.contains(HardwareDevice.DRIVE_MOTORS)) {
                if (drivePowersBuffered) {
                    double[] drivePowers = drivePowerBuffer.get(0);
                    if (drivePowers != null) {
                        for (int i = 0; i < 4; i++) {
                            driveMotors.get(i).setPower(drivePowers[i]);
                        }
                    }
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

            if(drivePowersBuffered){
                //HOPEFULLY by now main loop is waiting for calibration and not about to send drive powers lol
                drivePowerBuffer.remove(0);
            }

            fpsDebug.endIncrement();
            fpsDebug.update();
            dataBuffer.add(data);
            while (System.nanoTime()-startTime<MIN_WAIT_TIME);
            enabledDevices.clear();
            enabledDevices.addAll(this.enabledDevices);
            dataLogged = true;
        }
    }

    public ReadData newData(){
        while (!dataLogged);
        dataLogged = false;
        fpsDebug.queryFPS();
        ReadData data = dataBuffer.get(dataBuffer.size()-1);
        dataBuffer.remove(dataBuffer.size()-1);
        return data;
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

    public enum HardwareDevice {
        DRIVE_MOTORS,
        HUB_1_BULK,
        HUB_2_BULK,
        PIXYCAM,
        GYRO
    }
}