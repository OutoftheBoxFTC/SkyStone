package Hardware.HardwareDevices;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SmartMotor {
    private DcMotor motor;
    private double power;
    private boolean updated;

    /**
     * Creates a new Smart Motor. Smart Motors do not send duplicate power send commands
     * @param motor the DcMotor to assign to this instance
     */
    public SmartMotor(DcMotor motor){
        this.motor = motor;
        power = 0;
        updated = false;
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
        if(this.power != power){
            this.power = power;
            updated = true;
        }
        updated = true; //DELETE
    }

    /**
     * Gets the motor power
     * @return the motor power
     */
    public double getMotorPower(){
        return power;
    }

    /**
     * Updates the motor, setting the motor power if the set power is different
     */
    public void updateMotor(){
        if(updated){
            motor.setPower(power);
            updated = false;
        }
    }
}
