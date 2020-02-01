package Motion.Terminator;

import HardwareSystems.SensorData;
import math.Vector3;

public class VariedTripwireTerminator extends Terminator{
    double distance;
    int numFrames;
    int currNumberFrames;

    public VariedTripwireTerminator(Vector3 position, Vector3 target, SideLaserTerminator.DISTANCE distance, int numFrames) {
        super(position, target);
        this.distance = distance.getDistance();
        this.numFrames = numFrames;
        currNumberFrames = 0;
    }

    public VariedTripwireTerminator(Vector3 position, Vector3 target, double distance, int numFrames) {
        super(position, target);
        this.distance = distance;
        this.numFrames = numFrames;
        currNumberFrames = 0;
    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        double sensorDistance = data.getIntakeTripwire();
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
