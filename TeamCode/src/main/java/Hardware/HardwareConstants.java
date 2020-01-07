package Hardware;

import math.Vector2;
import math.Vector3;

/**
 * Constants for hardware devices
 */
public class HardwareConstants {
    //Servos
    public static final Vector2 LATCH_ON = new Vector2(0.25, 0.95);
    public static final Vector2 LATCH_OFF = new Vector2(0.75, 0.29);
    public static final Vector2 OPEN_INTAKE = new Vector2(0.95, 1-0.95);
    public static final Vector2 CLOSE_INTAKE = new Vector2(0.6, 1-0.6);
    public static final Vector2 LOCKED_INTAKE = new Vector2(0.45, 1-0.45);
    public static final double INTAKE_LATCH_ON = 0.85;
    public static final double INTAKE_LATCH_OFF = 0.3678;
    public static final Vector2 LIFT_REST = new Vector2(SQUARE_WAVE_TO_POSITION(910), SQUARE_WAVE_TO_POSITION(860));
    public static final Vector2 LIFT_OUT = new Vector2(SQUARE_WAVE_TO_POSITION(2015), SQUARE_WAVE_TO_POSITION(1915));
    public static final Vector2 LIFT_MID = new Vector2(SQUARE_WAVE_TO_POSITION(1165), SQUARE_WAVE_TO_POSITION(1095));
    public static final Vector2 LIFT_OUT_READY = new Vector2(SQUARE_WAVE_TO_POSITION(1740), SQUARE_WAVE_TO_POSITION(1650));
    public static final Vector2 LIFT_INTAKE = new Vector2(SQUARE_WAVE_TO_POSITION(1000), SQUARE_WAVE_TO_POSITION(935));
    public static final double LIFT_REST_OFFSET = -0.12;
    public static final double LIFT_OUT_OFFSET = -0.21;
    public static double SQUARE_WAVE_TO_POSITION(double wave){
        return (wave-500)/2000;
    }
}
