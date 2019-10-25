package motion.Terminator;

import hardware.ReadData;
import hardware.Hardware;
import hardware.Sensors.Pixycam;
import math.Vector3;

public class PixyTerminator extends Terminator {
    Hardware robot;
    boolean shouldTerminate;
    public PixyTerminator(Vector3 position, Vector3 target, Hardware robot) {
        super(position, target);
        this.robot = robot;
    }

    @Override
    public boolean shouldTerminate(ReadData data) {
        return (robot.getPixy().getY() > 100 && robot.getPixy().getY() < 180);
    }
}
