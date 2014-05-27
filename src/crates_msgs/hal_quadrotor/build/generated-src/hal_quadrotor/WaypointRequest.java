package hal_quadrotor;

public interface WaypointRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/WaypointRequest";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 yaw\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getYaw();
  void setYaw(double value);
}
