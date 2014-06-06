package hal_quadrotor;

public interface AnglesHeightRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/AnglesHeightRequest";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 roll\nfloat64 pitch\nfloat64 yaw\nfloat64 z\n";
  double getRoll();
  void setRoll(double value);
  double getPitch();
  void setPitch(double value);
  double getYaw();
  void setYaw(double value);
  double getZ();
  void setZ(double value);
}
