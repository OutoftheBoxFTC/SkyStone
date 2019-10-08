package motion.Terminator;

import hardware.ReadData;
import math.Vector3;

public class NullTerminator extends Terminator {
    public NullTerminator() {
        super(Vector3.ZERO(), Vector3.ZERO());
    }

    @Override
    public boolean shouldTerminate(ReadData data) {
        return false;
    }
}
