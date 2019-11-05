package Hardware.HardwareDevices;

import com.qualcomm.robotcore.hardware.Servo;

public class SmartServo {
    private Servo servo;
    private double position;
    private boolean updated;

    /**
     * Creates a new Smart Servos. Smart Servos do not send duplicate power send commands
     * @param servo the servo to assign to this instance
     */
    public SmartServo(Servo servo){
        this.servo = servo;
        position = 0;
        updated = false;
    }

    /**
     * Sets the servo position. Duplicate servo positions are not sent
     * @param position the position to set the servo to
     */
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

    /**
     * Gets the Servo associated with this class
     * @return the Servo associated with this class
     */
    public Servo getServo(){
        return servo;
    }
}
