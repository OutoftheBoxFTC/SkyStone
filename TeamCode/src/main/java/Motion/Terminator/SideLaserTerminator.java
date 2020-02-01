package Motion.Terminator;

import HardwareSystems.SensorData;
import math.Vector3;

public class SideLaserTerminator extends Terminator{
    SIDE side;
    double distance;
    int numFrames;
    int currNumberFrames;

    public SideLaserTerminator(Vector3 position, Vector3 target, SIDE side, DISTANCE distance, int numFrames) {
        super(position, target);
        this.side = side;
        this.distance = distance.getDistance();
        this.numFrames = numFrames;
        currNumberFrames = 0;
    }

    public SideLaserTerminator(Vector3 position, Vector3 target, SIDE side, double distance, int numFrames) {
        super(position, target);
        this.side = side;
        this.distance = distance;
        this.numFrames = numFrames;
        currNumberFrames = 0;
    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        double sensorDistance = (side == SIDE.LEFT) ? data.getLeft() : data.getRight();
        if(sensorDistance <= distance){
            currNumberFrames ++;
            if(currNumberFrames >= numFrames){
                return true;
            }
        }else{
            currNumberFrames = 0;
        }
        return false;
    }

    public enum SIDE{
        RIGHT,
        LEFT
    }

    public enum DISTANCE{
        CLOSE(2), MEDIUM(6), LONG(12);

        private int value;

        public int getDistance()
        {
            return this.value;
        }

        private DISTANCE(int value)
        {
            this.value = value;
        }
    }
}
