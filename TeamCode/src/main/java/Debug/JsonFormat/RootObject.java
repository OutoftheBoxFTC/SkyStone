package Debug.JsonFormat;

public class RootObject
    {
        public Coordinates Coordinates;
        public SensorIO SensorIO;
        public TelemetryData TelemetryData;
        public PathCoordinates PathCoordinates;
        public RootObject(Coordinates coordinates, SensorIO sensorIO, TelemetryData telemetryData, PathCoordinates PathCoordinates){
            this.Coordinates = coordinates;
            this.SensorIO = sensorIO;
            this.TelemetryData = telemetryData;
            this.PathCoordinates = PathCoordinates;
        }
    }