package Hardware.HardwareDevices;

import com.qualcomm.robotcore.hardware.Servo;

public class SmartServo {
    private Servo servo;
    private double position;
    private boolean updated;
    public SmartServo(Servo servo){
        this.servo = servo;
        position = 0;
        updated = false;
    }
    public void setPosition(double position){
        if(this.position != position){
            this.position = position;
            updated = true;
        }
    }

    /**
     * Sets the servo position, if the new position is different than the old one
     */
    public void updateServo(){
        if(updated){
            servo.setPosition(position);
            updated = false;
        }
    }

    public Servo getServo(){
        return servo;
    }
}
