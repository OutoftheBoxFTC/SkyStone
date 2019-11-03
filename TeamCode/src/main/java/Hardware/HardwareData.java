package Hardware;

import math.Vector2;
import math.Vector4;

public class HardwareData {
    private double timeStamp, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, latchRight, latchLeft, intakeLeft, intakeRight;
    private long timestamp;
    public HardwareData(long timestamp){
        this.timestamp = timestamp;
        frontLeftMotor = 0;
        frontRightMotor = 0;
        backLeftMotor = 0;
        backRightMotor = 0;
        latchRight = 0;
        latchLeft = 0;
        intakeRight = 0;
        intakeLeft = 0;
    }

    public void setMotorPowers(double frontLeftMotor, double frontRightMotor, double backLeftMotor, double backRightMotor){
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    public void setMotorPowers(Vector4 motorPowers){
        this.frontLeftMotor = motorPowers.getA();
        this.frontRightMotor = motorPowers.getB();
        this.backLeftMotor = motorPowers.getC();
        this.backRightMotor = motorPowers.getD();
    }

    public void setLatchServos(double leftLatch, double rightLatch){
        this.latchLeft = leftLatch;
        this.latchRight = rightLatch;
    }

    public void setIntakePowers(double intakeLeft, double intakeRight){
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
    }

    public void setTimestamp(long timestamp){
        this.timestamp = timestamp;
    }
    /**
    Returns the motor powers to the drive motors
    @return A vector4 containing the powers in the order [frontLeft, frontRight, backLeft, backRight]
     */
    public Vector4 getMotorPowers(){
        Vector4 motorPowers = Vector4.ZERO();
        motorPowers.setA(frontLeftMotor);
        motorPowers.setB(frontRightMotor);
        motorPowers.setC(backLeftMotor);
        motorPowers.setD(backRightMotor);
        return motorPowers;
    }

    /**
     * Returns the latch positions
     * @return A vector2 containing the latch positions in the order [left, right]
     */
    public Vector2 getLatchPositions(){
        Vector2 positions = Vector2.ZERO();
        positions.setA(latchLeft);
        positions.setB(latchRight);
        return positions;
    }

    /**
     * Returns the motor powers to the intake
     * @return A vector2 containing the intake powers in the order [left, right]
     */
    public Vector2 getIntakePowers(){
        Vector2 positions = Vector2.ZERO();
        positions.setA(intakeLeft);
        positions.setB(intakeRight);
        return positions;
    }

    public long getTimestamp(){
        return timestamp;
    }
}
