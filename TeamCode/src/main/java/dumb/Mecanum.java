package dumb;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Mecanum {
    OpMode opmode;
    DcMotor FL, FR, BL, BR;
    Servo servo;
    BNO055IMU gyro;
    float globalangle;
    public Mecanum(OpMode opmode){
        this.opmode = opmode;
    }

    public void init(){
        FL = opmode.hardwareMap.get(DcMotor.class, "a");
        FR = opmode.hardwareMap.get(DcMotor.class, "b");
        BL = opmode.hardwareMap.get(DcMotor.class, "c");
        BR = opmode.hardwareMap.get(DcMotor.class, "d");
        servo = opmode.hardwareMap.get(Servo.class, "servo");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        gyro = opmode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        globalangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    public void drive(double forward, double strafe, double rot){
        opmode.telemetry.addData("test", gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + " " + gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle + " " + gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        opmode.telemetry.update();
        double angle = (gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle - globalangle) + (Math.atan2(forward, strafe));
        double newFor = Math.sqrt((strafe * strafe) + (forward * forward)) * Math.sin(angle);
        double newStr = Math.sqrt((strafe * strafe) + (forward * forward)) * Math.cos(angle);
        FL.setPower((-newFor) + newStr + rot);
        FR.setPower(newFor + newStr + rot);
        BL.setPower((-newFor) - newStr + rot);
        BR.setPower(newFor - newStr + rot);
    }

    public void driveRaw(double newFor, double newStr, double rot){
        FL.setPower((-newFor) + newStr + rot);
        FR.setPower(newFor + newStr + rot);
        BL.setPower((-newFor) - newStr + rot);
        BR.setPower(newFor - newStr + rot);
    }

    public Servo getServo(){
        return servo;
    }
}
