package HardwareSystems.HardwareDevices;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SmartMotor {
    private DcMotor motor;
    private double power, previousPower;

    /**
     * Creates a new Smart Motor. Smart Motors do not send duplicate power send commands
     * @param motor the DcMotor to assign to this instance
     */
    public SmartMotor(DcMotor motor){
        this.motor = motor;
        power = 0;
        previousPower = 0;
    }

    /**
     * Gets the DcMotor associated with this class
     * @return the DcMotor associated with this class
     */
    public DcMotor getMotor(){
        return motor;
    }

    /**
     * Sets the motor power. If the power has not changed a new command is not sent to the motor
     * @param power the power to set [-1, 1]
     */
    public void setPower(double power){
        if(Math.abs(power-previousPower) > 0){
            previousPower = power;
            this.power = power;
            motor.setPower(power);
        }
    }

    /**
     * Gets the motor power
     * @return the motor power
     */
    public double getMotorPower(){
        return power;
    }
}
