package Debug.JsonFormat;

public class RootObject
    {
        public Coordinates Coordinates;
        public SensorIO SensorIO;
        public TelemetryData TelemetryData;
        public RootObject(Coordinates coordinates, SensorIO sensorIO, TelemetryData telemetryData){
            this.Coordinates = coordinates;
            this.SensorIO = sensorIO;
            this.TelemetryData = telemetryData;
        }
    }