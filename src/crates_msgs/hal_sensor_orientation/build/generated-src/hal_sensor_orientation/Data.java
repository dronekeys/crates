package hal_sensor_orientation;

public interface Data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_orientation/Data";
  static final java.lang.String _DEFINITION = "float64 t       # Time at which measurement was taken\nfloat64 roll   \t# Roll  X\nfloat64 pitch  \t# Pitch Y\nfloat64 yaw    \t# Yaw   Z\nfloat64 p   \t# Angvel X\nfloat64 q  \t\t# Angvel Y\nfloat64 r   \t# Angvel Z";
  double getT();
  void setT(double value);
  double getRoll();
  void setRoll(double value);
  double getPitch();
  void setPitch(double value);
  double getYaw();
  void setYaw(double value);
  double getP();
  void setP(double value);
  double getQ();
  void setQ(double value);
  double getR();
  void setR(double value);
}
