package hal_sensor_compass;

public interface ConfigureRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_compass/ConfigureRequest";
  static final java.lang.String _DEFINITION = "float64 samprate\nfloat64 sendrate\n";
  double getSamprate();
  void setSamprate(double value);
  double getSendrate();
  void setSendrate(double value);
}
