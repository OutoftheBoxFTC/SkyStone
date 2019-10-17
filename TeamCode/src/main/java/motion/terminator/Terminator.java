package motion.terminator;

import hardware.ReadData;

public abstract class Terminator {
    public abstract boolean shouldTerminate(ReadData data);
}
