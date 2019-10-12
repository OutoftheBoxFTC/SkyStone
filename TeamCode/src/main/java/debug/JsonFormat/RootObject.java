package debug.JsonFormat;

public class RootObject
    {
        public debug.JsonFormat.Coordinates Coordinates;
        public debug.JsonFormat.SensorIO SensorIO;
        public debug.JsonFormat.TelemetryData TelemetryData;
        public RootObject(debug.JsonFormat.Coordinates coordinates, debug.JsonFormat.SensorIO sensorIO, debug.JsonFormat.TelemetryData telemetryData){
            this.Coordinates = coordinates;
            this.SensorIO = sensorIO;
            this.TelemetryData = telemetryData;
        }
    }