package motion.Terminator;

import hardware.ReadData;
import math.Vector3;

public abstract class Terminator {
    Vector3 position, target;
    public Terminator(Vector3 position, Vector3 target){
        this.position = position;
        this.target = target;
    }
    public abstract boolean shouldTerminate(ReadData data);

    public static Terminator NONE(){
        return new NullTerminator();
    }
}
