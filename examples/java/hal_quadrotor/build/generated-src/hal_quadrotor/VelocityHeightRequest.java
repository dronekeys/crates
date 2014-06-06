package hal_quadrotor;

public interface VelocityHeightRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/VelocityHeightRequest";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 u\nfloat64 v\nfloat64 z\nfloat64 yaw\n";
  double getU();
  void setU(double value);
  double getV();
  void setV(double value);
  double getZ();
  void setZ(double value);
  double getYaw();
  void setYaw(double value);
}
