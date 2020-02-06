package HardwareSystems.HardwareDevices;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class SmartServo {
    private ServoImplEx servo;
    private double position;

    /**
     * Creates a new Smart Servos. Smart Servos do not send duplicate power send commands
     * @param servo the servo to assign to this instance
     */
    public SmartServo(Servo servo){
        this.servo = (ServoImplEx) servo;
        position = 0;
        this.servo.setPwmDisable();
    }

    /**
     * Sets the servo position. Duplicate servo positions are not sent
     * @param position the position to set the servo to
     */
    public void setPosition(double position){
        if(this.position != position){
            this.position = position;
            if(position <= 1) {
                servo.setPosition(position);
            }else{
                if(!servo.isPwmEnabled()){
                    servo.setPwmEnable();
                }
                if(this.position != position){
                    double min = servo.getPwmRange().usPulseLower;
                    double max = servo.getPwmRange().usPulseUpper;
                    servo.setPosition((position-min)/max);
                }
            }
        }
    }

    public void disableServo(){
        if(servo.isPwmEnabled()) {
            servo.setPwmDisable();
        }else{
            servo.setPwmEnable();
        }
    }

    /**
     * Gets the Servo associated with this class
     * @return the Servo associated with this class
     */
    public ServoImplEx getServo(){
        return servo;
    }
}
