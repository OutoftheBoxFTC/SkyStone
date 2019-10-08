package debug.JsonFormat;

public class RootObject
    {
        public Coordinates Coordinates;
        public SensorIO SensorIO;
        public debug.JsonFormat.TelemetryData TelemetryData;
        public RootObject(Coordinates coordinates, SensorIO sensorIO, debug.JsonFormat.TelemetryData telemetryData){
            this.Coordinates = coordinates;
            this.SensorIO = sensorIO;
            this.TelemetryData = telemetryData;
        }
    }