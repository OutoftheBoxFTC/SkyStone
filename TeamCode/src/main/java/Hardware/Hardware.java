package Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import Hardware.HardwareDevices.SmartMotor;
import Hardware.HardwareDevices.SmartServo;
import Hardware.Sensors.Pixycam;
import math.Vector2;
import math.Vector3;
import math.Vector4;

public class Hardware {
    private SmartMotor frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight, liftMotorLeft, liftMotorRight;
    private SmartServo leftLatch, rightLatch, intakeServoLeft, intakeServoRight, liftServoLeft, liftServoRight, intakeLatch;
    private OpMode opmode;
    private Telemetry telemetry;
    private ArrayList<HardwareDevices> enabledDevices;
    private CalibrationSystem calibration;
    private BNO055IMU imu;
    private Pixycam pixy;
    private OpenCvCamera liftCam;
    private OpenCvPipeline pipeline;


    /**
     * Creats a new Hardware
     * @param opmode the opmode
     * @param telemetry telemetry for debug
     */
    public Hardware(OpMode opmode, Telemetry telemetry){
        this.opmode = opmode;
        this.telemetry = telemetry;
        enabledDevices = new ArrayList<>();
        pipeline = null;
    }

    /**
     * initializes the hardware, assigning enabled devices from the hardware map
     */
    public void init(){
        HardwareMap map = opmode.hardwareMap;
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)){
            frontLeft = new SmartMotor(getOrNull(map, DcMotor.class, "ll"));
            frontRight = new SmartMotor(getOrNull(map, DcMotor.class, "lr"));
            backLeft = new SmartMotor(getOrNull(map, DcMotor.class, "tl"));
            backRight = new SmartMotor(getOrNull(map, DcMotor.class, "tr"));
            frontLeft.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(enabledDevices.contains(HardwareDevices.ODOMETRY)){
            //odometryRightMotor = getOrNull(map, DcMotor.class, "odometryR");
            //odometryLeftMotor = getOrNull(map, DcMotor.class, "odometryL");
        }
        if(enabledDevices.contains(HardwareDevices.LIFT_CAM)){
            int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
            liftCam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            liftCam.openCameraDevice();
        }
        if(enabledDevices.contains(HardwareDevices.LATCH_SERVOS)){
            leftLatch = new SmartServo(getOrNull(map, Servo.class, "leftLatch"));
            rightLatch = new SmartServo(getOrNull(map, Servo.class, "rightLatch"));
        }
        if(enabledDevices.contains(HardwareDevices.INTAKE)){
            intakeLeft = new SmartMotor(getOrNull(map, DcMotor.class, "leftIntake"));
            intakeRight = new SmartMotor(getOrNull(map, DcMotor.class, "rightIntake"));
            intakeServoLeft = new SmartServo(getOrNull(map, Servo.class, "leftIntakeServo"));
            intakeServoRight = new SmartServo(getOrNull(map, Servo.class, "rightIntakeServo"));
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)){
            imu = getOrNull(map, BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
        }
        if(enabledDevices.contains(HardwareDevices.LEFT_PIXY)){
            pixy = getOrNull(map, Pixycam.class, "pixyLeft");
        }else if(enabledDevices.contains(HardwareDevices.RIGHT_PIXY)){
            pixy = getOrNull(map, Pixycam.class, "pixyRight");
        }
        if(enabledDevices.contains(HardwareDevices.LIFT_SERVOS)){
            liftServoLeft = new SmartServo(getOrNull(map, Servo.class, "liftServoL"));
            liftServoRight = new SmartServo(getOrNull(map, Servo.class, "liftServoR"));
        }
        if(enabledDevices.contains(HardwareDevices.LIFT_MOTORS)){
            liftMotorLeft = new SmartMotor(getOrNull(map, DcMotor.class, "liftMotorL"));
            liftMotorRight = new SmartMotor(getOrNull(map, DcMotor.class, "liftMotorR"));
            liftMotorLeft.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            calibration.setLift(liftMotorLeft.getMotor().getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.INTAKE_LATCH)){
            intakeLatch = new SmartServo(getOrNull(map, Servo.class, "intakeLatch"));
        }
    }

    /**
     * Calibrates all enabled devices. Must be called before update
     */
    public void calibrate(){
        calibration = new CalibrationSystem();
        if(enabledDevices.contains(HardwareDevices.DRIVE_MOTORS)) {
            calibration.setOdometryEncoders(-intakeLeft.getMotor().getCurrentPosition(), intakeRight.getMotor().getCurrentPosition(), frontLeft.getMotor().getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)) {
            Orientation orientation = imu.getAngularOrientation();
            double yaw = orientation.firstAngle;
            double tau = Math.PI * 2;
            calibration.setGyro(((yaw % tau) + tau) % tau);
        }
        if(enabledDevices.contains(HardwareDevices.LIFT_CAM)){
            if(pipeline != null) {
                liftCam.setPipeline(pipeline);
            }
            liftCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        }
    }

    public void calibrate(CalibrationSystem calibration){
        this.calibration = calibration;
    }

    /**
     * updates all sensors and hardware devices from the HardwareData
     * @param data HardwareData class to assign all hardware devices values
     * @return SensorData class containing new sensor data
     */
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
            sensors.setOdometryEncoders(-intakeLeft.getMotor().getCurrentPosition(), intakeRight.getMotor().getCurrentPosition(), frontLeft.getMotor().getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.INTAKE)){
            intakeLeft.setPower(data.getIntakePowers().getA());
            intakeRight.setPower(data.getIntakePowers().getB());
            intakeServoLeft.setPosition(data.getIntakeServos().getA());
            intakeServoRight.setPosition(data.getIntakeServos().getB());
        }
        if(enabledDevices.contains(HardwareDevices.GYRO)) {
            Orientation orientation = imu.getAngularOrientation();
            double yaw = orientation.firstAngle;
            double tau = Math.PI * 2;
            sensors.setGyro(((yaw % tau) + tau) % tau);
        }
        if(enabledDevices.contains(HardwareDevices.LEFT_PIXY) || enabledDevices.contains(HardwareDevices.RIGHT_PIXY)){
            sensors.setPixy(pixy.getCoordinateColor());
        }
        if(enabledDevices.contains(HardwareDevices.LIFT_MOTORS)){
            liftMotorLeft.setPower(data.getLiftMotors());
            liftMotorRight.setPower(data.getLiftMotors());
            sensors.setLift(liftMotorLeft.getMotor().getCurrentPosition());
        }
        if(enabledDevices.contains(HardwareDevices.LIFT_SERVOS)){
            liftServoLeft.setPosition(data.getLiftServo().getA());
            liftServoRight.setPosition(data.getLiftServo().getB());

        }
        if(enabledDevices.contains(HardwareDevices.INTAKE_LATCH)){
            intakeLatch.setPosition(data.getIntakeLatch());
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

    /**
     * Enables all hardware devices
     */
    public void enableAll(){
        enabledDevices.add(HardwareDevices.DRIVE_MOTORS);
        enabledDevices.add(HardwareDevices.LATCH_SERVOS);
        enabledDevices.add(HardwareDevices.INTAKE);
        enabledDevices.add(HardwareDevices.GYRO);
        enabledDevices.add(HardwareDevices.ODOMETRY);
        enabledDevices.add(HardwareDevices.LIFT_SERVOS);
        enabledDevices.add(HardwareDevices.LIFT_MOTORS);
    }

    /**
     * Enables specific hardware deivice
     * @param device device to enable
     */
    public void enableDevice(HardwareDevices device){
        if(!enabledDevices.contains(device)){
            enabledDevices.add(device);
        }
    }

    /**
     * Disables specific hardware device
     * @param device the device to disable
     */
    public void disableDevice(HardwareDevices device){
        if(enabledDevices.contains(device)){
            enabledDevices.remove(device);
        }
    }

    public Vector3 getAllGyroAngles(){
        Orientation orientation = imu.getAngularOrientation();
        double yaw = orientation.firstAngle;
        double tau = Math.PI * 2;
        double firstRot = (((yaw % tau) + tau) % tau);
        yaw = orientation.secondAngle;
        double secondRot = (((yaw % tau) + tau) % tau);
        yaw = orientation.thirdAngle;
        double thirdRot = (((yaw % tau) + tau) % tau);
        return new Vector3(firstRot, secondRot, thirdRot);
    }

    public CalibrationSystem getCalibration(){
        return calibration;
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
    }

    public void stop(){
        liftCam.stopStreaming();
        liftCam.closeCameraDevice();
    }

    /**
     * Enumaltion of all HardwareDevices
     */
    public enum HardwareDevices{
        DRIVE_MOTORS,
        LATCH_SERVOS,
        INTAKE,
        GYRO,
        ODOMETRY,
        LEFT_PIXY,
        RIGHT_PIXY,
        LIFT_SERVOS,
        LIFT_MOTORS,
        INTAKE_LATCH,
        LIFT_CAM
    }
}
