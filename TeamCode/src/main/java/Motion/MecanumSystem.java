package Motion;

import math.Vector3;
import math.Vector4;

public class MecanumSystem {
    /**
     * Translates x y r to motor powers
     * @param coords a vector3 containing {x, y, r}
     * @return motor powers in a vector4
     */
    public static Vector4 translate(Vector3 coords){
        return new Vector4(-coords.getB() + coords.getA() - coords.getC(), coords.getB() + coords.getA() - coords.getC(), -coords.getB() - coords.getA() - coords.getC(), coords.getB() - coords.getA() - coords.getC());
    }
}
