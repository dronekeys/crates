package hal_sensor_altimeter;

public interface Data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_altimeter/Data";
  static final java.lang.String _DEFINITION = "float64 t     # Time at which measurement was taken\nfloat64 z     # Barometric altitude\nfloat64 w     # Barometric velocity";
  double getT();
  void setT(double value);
  double getZ();
  void setZ(double value);
  double getW();
  void setW(double value);
}
