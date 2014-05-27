package hal_sensor_imu;

public interface Data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_imu/Data";
  static final java.lang.String _DEFINITION = "float64 t    # Time at which measurement was taken\nfloat64 p    # Body-frame angular velocity X\nfloat64 q    # Body-frame angular velocity Y\nfloat64 r    # Body-frame angular velocity Z\nfloat64 du   # Body-frame acceleration X\nfloat64 dv   # Body-frame acceleration Y\nfloat64 dw   # Body-frame acceleration Z";
  double getT();
  void setT(double value);
  double getP();
  void setP(double value);
  double getQ();
  void setQ(double value);
  double getR();
  void setR(double value);
  double getDu();
  void setDu(double value);
  double getDv();
  void setDv(double value);
  double getDw();
  void setDw(double value);
}
