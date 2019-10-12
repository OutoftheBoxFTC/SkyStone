package hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

public class SmartMotor {
    private ExpansionHubMotor motor;

    private double previousPower;
    private DcMotor.RunMode previousRunMode;
    private DcMotor.ZeroPowerBehavior previousZeroPowerBehavior;

    public SmartMotor(ExpansionHubMotor motor){
        this.motor = motor;
        if(motor != null){
            previousRunMode = motor.getMode();
            previousZeroPowerBehavior = motor.getZeroPowerBehavior();
        }
    }

    public void setPower(double power){
        if(Math.abs(power-previousPower)>=0.005){
            previousPower = power;
            motor.setPower(power);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behaviour){
        if(!previousZeroPowerBehavior.equals(behaviour)){
            previousZeroPowerBehavior = behaviour;
            motor.setZeroPowerBehavior(behaviour);
        }
    }

    public void setMode(DcMotor.RunMode mode){
        if(!mode.equals(previousRunMode)){
            previousRunMode = mode;
            motor.setMode(mode);
        }
    }

    public ExpansionHubMotor getMotor() {
        return motor;
    }
}
