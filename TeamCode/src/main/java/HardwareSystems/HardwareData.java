package HardwareSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import math.Vector2;
import math.Vector4;

public class HardwareData {
    private double timeStamp, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, latchRight, latchLeft, intakeLeft, intakeRight, intakeServoLeft, intakeServoRight, liftMotor, liftServo, intakeLatch, liftServoOffset, capstoneLatch;
    private RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    Vector2 intakeServos;
    private long timestamp;

    /**
     * Creates a new Hardware Data instance
     * @param timestamp the current timestamp for debug
     */
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
        intakeServoLeft = 0;
        intakeServoRight = 0;
        capstoneLatch = 0;
    }

    /**
     * Sets the motor powers
     * @param frontLeftMotor
     * @param frontRightMotor
     * @param backLeftMotor
     * @param backRightMotor
     */

    public void setMotorPowers(double frontLeftMotor, double frontRightMotor, double backLeftMotor, double backRightMotor){
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    /**
     * Sets the motor powers
     * @param motorPowers a vector4 containing {frontLeft, frontRight, backLeft, backRight}
     */

    public void setMotorPowers(Vector4 motorPowers){
        this.frontLeftMotor = motorPowers.getA();
        this.frontRightMotor = motorPowers.getB();
        this.backLeftMotor = motorPowers.getC();
        this.backRightMotor = motorPowers.getD();
    }

    /**
     * Sets the latch servo positions
     * @param leftLatch
     * @param rightLatch
     */

    public void setLatchServos(double leftLatch, double rightLatch){
        this.latchLeft = leftLatch;
        this.latchRight = rightLatch;
    }

    /**
     * Sets the latch servo positions
     * @param positions a vector2 containing {left, right}
     */

    public void setLatchServos(Vector2 positions){
        this.latchLeft = positions.getA();
        this.latchRight = positions.getB();
    }

    /**
     * Sets the intake powers
     * @param intakeLeft
     * @param intakeRight
     */

    public void setIntakePowers(double intakeLeft, double intakeRight){
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
    }

    /**
     * Sets the intake powers
     * @param power the power to set both intake motors to
     */

    public void setIntakePowers(double power){
        this.intakeLeft = power;
        this.intakeRight = power;
    }

    /**
     * Sets the timestamp for debug
     * @param timestamp
     */

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

    /**
     * Sets the intake servos
     * @param left the left servo position
     * @param right the right servo position
     */

    public void setIntakeServos(double left, double right){
        intakeServoLeft = left;
        intakeServoRight = right;
    }

    /**
     * Sets the intake servos
     * @param positions a vector2 containing the latch positions in the order [left, right]
     */

    public void setIntakeServos(Vector2 positions){
        intakeServoLeft = positions.getA();
        intakeServoRight = positions.getB();
    }

    public Vector2 getIntakeServos(){
        return new Vector2(intakeServoLeft, intakeServoRight);
    }

    public void setLiftMotors(double power){
        liftMotor = power;
    }

    public double getLiftMotors(){
        return liftMotor;
    }

    public void setLiftServo(Vector2 position){
        liftServo = position.getA();
        liftServoOffset = position.getB() - position.getA();
    }
    public void setLiftServo(Vector2 position, double trash){
        liftServo = position.getA();
        liftServoOffset = position.getB() - position.getA();
    }
    public void setLiftServo(double position){
        liftServo = position;
    }
    public void setLiftServo(double position, double offset){
        liftServo = position;
        liftServoOffset = offset;
    }

    public Vector2 getLiftServo(){
        return new Vector2(liftServo, liftServo + liftServoOffset);
    }

    public double getLiftServoDouble(){
        return liftServo;
    }

    public void setIntakeLatch(double position){
        intakeLatch = position;
    }

    public double getIntakeLatch(){
        return intakeLatch;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        this.pattern = pattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){
        return pattern;
    }

    public void setCapstoneLatch(double position){
        this.capstoneLatch = position;
    }

    public double getCapstoneLatch(){
        return capstoneLatch;
    }

    /**
     * Gets the timestamp
     * @return the timestamp
     */

    public long getTimestamp(){
        return timestamp;
    }
}
