package hal_sensor_gnss;

public interface Data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_gnss/Data";
  static final java.lang.String _DEFINITION = "float64 t     # Time at which measurement was taken\nfloat64 x     # Navigation frame position X\nfloat64 y     # Navigation frame position Y\nfloat64 z     # Navigation frame position Z\nfloat64 u     # Navigation frame velocity X\nfloat64 v     # Navigation frame velocity Y\nfloat64 w     # Navigation frame velocity Z";
  double getT();
  void setT(double value);
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getU();
  void setU(double value);
  double getV();
  void setV(double value);
  double getW();
  void setW(double value);
}
