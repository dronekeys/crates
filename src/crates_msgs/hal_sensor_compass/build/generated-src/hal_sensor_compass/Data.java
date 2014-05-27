package hal_sensor_compass;

public interface Data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_compass/Data";
  static final java.lang.String _DEFINITION = "float64 t     # Time at which measurement was taken\nfloat64 m_x     # Body frame magnetic field strength X\nfloat64 m_y     # Body frame magnetic field strength Y\nfloat64 m_z     # Body frame magnetic field strength Z\n";
  double getT();
  void setT(double value);
  double getMX();
  void setMX(double value);
  double getMY();
  void setMY(double value);
  double getMZ();
  void setMZ(double value);
}
