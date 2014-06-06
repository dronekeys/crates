package hal_quadrotor;

public interface VelocityRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/VelocityRequest";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 u\nfloat64 v\nfloat64 w\nfloat64 yaw\n";
  double getU();
  void setU(double value);
  double getV();
  void setV(double value);
  double getW();
  void setW(double value);
  double getYaw();
  void setYaw(double value);
}
