package Motion;

import math.Vector3;
import math.Vector4;

/**
 * xyyx
 * +-+-
 */
public class MecanumSystem {
    public static Vector4 translate(Vector3 coords){
        return new Vector4(coords.getA() + coords.getC(), coords.getB() - coords.getC(), coords.getB() + coords.getC(), coords.getA() - coords.getC());
    }
}
