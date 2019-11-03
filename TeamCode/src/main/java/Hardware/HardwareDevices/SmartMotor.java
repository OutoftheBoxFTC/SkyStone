package Hardware.HardwareDevices;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SmartMotor {
    private DcMotor motor;
    private double power;
    private boolean updated;
    public SmartMotor(DcMotor motor){
        this.motor = motor;
        power = 0;
        updated = false;
    }
    public DcMotor getMotor(){
        return motor;
    }
    public void setPower(double power){
        if(this.power != power){
            this.power = power;
            updated = true;
        }
    }
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
