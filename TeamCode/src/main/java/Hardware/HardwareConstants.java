package Hardware;

import math.Vector2;
import math.Vector3;

/**
 * Constants for hardware devices
 * All direction are assuming you are behind the robot (on the side where we outtake the block)
 */
public class HardwareConstants {
    //Servos
    public static final Vector2 LATCH_ON = new Vector2(0.1, 0.95); //The servo positions for latching ON the foundation. It is in the order {left, right}
    public static final Vector2 LATCH_OFF = new Vector2(0.75, 0.24); //The servo positions for latching OFF the foundation. It is in the order {left, right}
    public static final Vector2 OPEN_INTAKE = new Vector2(0.95, 1-0.95); //The servo positions for OPENING the intake. It is in the order {left, right}
    public static final Vector2 CLOSE_INTAKE = new Vector2(0.4, 1-0.4); //The servo positions for CLOSING the intake. It is in the order {left, right}
    public static final Vector2 CLOSE_INTAKE_TELEOP = new Vector2(0.658, 1-0.608); //The servo positions for CLOSING the intake IN TELEOP. It is in the order {left, right}
    public static final double INTAKE_LATCH_ON = 0.65; //The servo position for the latch on the four bar lift to latch ON to the block
    public static final double INTAKE_LATCH_OFF = 0.3678; //The servo position for the latch on the four bar lift to latch OFF of the block
    public static final Vector2 LIFT_REST = new Vector2(SQUARE_WAVE_TO_POSITION(910), SQUARE_WAVE_TO_POSITION(830)); //The servo position for the four bar servos when in the REST position, or all the way forward. It is in the order {left, right}
    public static final Vector2 LIFT_INTAKE = new Vector2(SQUARE_WAVE_TO_POSITION(1000), SQUARE_WAVE_TO_POSITION(935)); //The servo position for the four bar servos when in the INTAKE position, or the position when intaking. It is in the order {
    public static final Vector2 LIFT_SCORING_POSITION = new Vector2(SQUARE_WAVE_TO_POSITION(1820), SQUARE_WAVE_TO_POSITION(1730)); //The servo position for the four bar servos when in the SCORING position. It is in the order {left, right}
    public static final Vector2 LIFT_OUT = new Vector2(SQUARE_WAVE_TO_POSITION(2015), SQUARE_WAVE_TO_POSITION(2000)); //The servo position for the four bar servos when in the OUT position, or when the arm is all the way out on the outtake side. It is in the order {left, right}
    public static final double CAPSTONE_LATCH_ON = SQUARE_WAVE_TO_POSITION(1800);
    public static final double CAPSTONE_LATCH_OFF = SQUARE_WAVE_TO_POSITION(900);
    public static double SQUARE_WAVE_TO_POSITION(double wave){
        //Converts a square wave value (500 to 2500) to a servo position (0 to 1)
        return (wave-500)/2000;
    }
}
