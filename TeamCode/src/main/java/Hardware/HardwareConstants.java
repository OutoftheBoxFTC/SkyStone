package Hardware;

import math.Vector2;

/**
 * Constants for hardware devices
 */
public class HardwareConstants {
    //Servos
    public static final Vector2 LATCH_ON = new Vector2(0.29, 0.95);
    public static final Vector2 LATCH_OFF = new Vector2(0.95, 0.29);
    public static final Vector2 OPEN_INTAKE = new Vector2(0.95, 1-0.95);
    public static final Vector2 CLOSE_INTAKE = new Vector2(0.708, 1-0.708);
    public static final Vector2 LOCKED_INTAKE = new Vector2(0.5, 1-0.5);
}
