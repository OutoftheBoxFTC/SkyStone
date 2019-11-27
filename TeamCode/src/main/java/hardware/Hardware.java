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

    private double[] drivePowers, intakePowers, intakeServos, liftPowers, liftServos;

    private SmartMotor leadingLeft, leadingRight, trailingLeft, trailingRight, leftIntake, rightIntake, leftLift, rightLift;
    private ExpansionHubServo leftServo, rightServo, lowerServo, upperServo;
    private ExpansionHubEx hub1, hub2;
    private OpenCvCamera wc1;
    private Pixycam leftPixy, rightPixy;


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
        intakeServos = new double[2];
        liftPowers = new double[2];
        liftServos = new double[2];

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
        if(registeredDevices.contains(HardwareDevice.PIXY_LEFT)){
            leftPixy = getOrNull(map, Pixycam.class, "leftPixy");
        }
        if(registeredDevices.contains(HardwareDevice.PIXY_RIGHT)){
            rightPixy = getOrNull(map, Pixycam.class, "rightPixy");
        }
        if(registeredDevices.contains(HardwareDevice.DRIVE_MOTORS)) {
            leadingLeft = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "ll"));
            leadingRight = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "lr"));
            trailingLeft = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "tl"));
            trailingRight = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "tr"));

            leadingRight.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            trailingRight.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            driveMotors.add(leadingLeft);
            driveMotors.add(leadingRight);
            driveMotors.add(trailingLeft);
            driveMotors.add(trailingRight);
        }
        if(registeredDevices.contains(HardwareDevice.INTAKE_MOTORS)){
            leftIntake = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "leftIntake"));
            rightIntake = new SmartMotor((ExpansionHubMotor) getOrNull(map.dcMotor, "rightIntake"));
        }
        if(registeredDevices.contains(HardwareDevice.INTAKE_SERVOS)){
            leftServo = (ExpansionHubServo) getOrNull(map.servo, "leftIntakeServo");
            rightServo = (ExpansionHubServo) getOrNull(map.servo, "rightIntakeServo");
        }
        if(registeredDevices.contains(HardwareDevice.LIFT_MOTORS)){
            leftLift = (SmartMotor) getOrNull(map.dcMotor, "leftLift");
            rightLift = (SmartMotor) getOrNull(map.dcMotor, "rightLift");
        }
        if(registeredDevices.contains(HardwareDevice.GYRO)) {
            imu = getOrNull(map, BNO055IMU.class, "imu");
            if(imu != null) {
                initIMU();
            }
        }
        if(registeredDevices.contains(HardwareDevice.LIFT_SERVOS)){
            lowerServo = (ExpansionHubServo) getOrNull(map.servo, "lowerServo");
            upperServo = (ExpansionHubServo) getOrNull(map.servo, "upperServo");
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
        if(registeredDevices.contains(HardwareDevice.INTAKE_SERVOS)){
            intakeServos(0, 0);
        }
        if(registeredDevices.contains(HardwareDevice.LIFT_SERVOS)){
            upperServo.setPosition(0);
            lowerServo.setPosition(0);
        }
        if(registeredDevices.contains(HardwareDevice.INTAKE_SERVOS)){
            rightIntake.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void intake(double left, double right){
        intakePowers = new double[]{left, right};
    }

    public void intakeServos(double left, double right){
        intakeServos = new double[]{1-left, right};
    }

    public void lift(double upper, double lower){
        liftPowers = new double[]{upper, lower};
    }

    public ReadData update() {
        fpsDebug.startIncrement();
        ReadData data = new ReadData(calibration);

        if(enabledDevices.contains(HardwareDevice.DRIVE_MOTORS)) {
            double[] drivePowers = this.drivePowers;
            if (drivePowers != null) {
                for (int i = 0; i < 4; i++) {
                    driveMotors.get(i).setPower(drivePowers[i]);
                }
            }
        }
        data.updateDrive();
        if(enabledDevices.contains(HardwareDevice.INTAKE_MOTORS)){
            if(intakePowers != null){
                leftIntake.setPower(intakePowers[0]);
                rightIntake.setPower(intakePowers[1]);
            }
        }
        if(enabledDevices.contains(HardwareDevice.INTAKE_SERVOS)){
            if(intakeServos != null){
                leftServo.setPosition(intakeServos[0]);
                rightServo.setPosition(intakeServos[1]);
            }
        }
        if(enabledDevices.contains(HardwareDevice.LIFT_MOTORS)){
            if(intakePowers != null){
                leftLift.setPower(liftPowers[0]);
                rightLift.setPower(liftPowers[1]);
            }
        }

        if(enabledDevices.contains(HardwareDevice.PIXY_LEFT)){
            data.addLeftPixy(leftPixy);
        }
        if(enabledDevices.contains(HardwareDevice.PIXY_RIGHT)){
            data.addRightPixy(rightPixy);
        }
        if(enabledDevices.contains(HardwareDevice.GYRO)){
            data.addGyro(imu);
        }
        if(enabledDevices.contains(HardwareDevice.HUB_1_BULK)) {
            RevBulkData rawData = hub1.getBulkInputData();
            data.addHub1BulkData(rawData);
        }
        if(enabledDevices.contains(HardwareDevice.HUB_2_BULK)){
            RevBulkData rawData = hub2.getBulkInputData();
            data.addHub2BulkData(rawData);
        }
        fpsDebug.endIncrement();
        fpsDebug.update();
        fpsDebug.queryFPS();
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

    public Hardware registerDevices(HardwareDevice... devices){
        for(HardwareDevice device : devices){
            registerDevice(device);
        }
        return this;
    }

    public Hardware enableDevice(HardwareDevice device){
        if(registeredDevices.contains(device)) {
            enabledDevices.add(device);
        }
        return this;
    }

    public Hardware disableDevice(HardwareDevice device){
        enabledDevices.remove(device);
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
        PIXY_LEFT,
        PIXY_RIGHT,
        GYRO,
        LIFT_MOTORS,
        LIFT_SERVOS
    }
}