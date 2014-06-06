package hal_quadrotor;

public interface Control extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/Control";
  static final java.lang.String _DEFINITION = "float64 t\t    \t# Time stamp\nfloat64 roll     \t# Body-frame X ROLL\nfloat64 pitch     \t# Body-frame Y PITCH\nfloat64 yaw     \t# Body-frame Z YAW\nfloat64 throttle    # Body-frame THROTTLE\n";
  double getT();
  void setT(double value);
  double getRoll();
  void setRoll(double value);
  double getPitch();
  void setPitch(double value);
  double getYaw();
  void setYaw(double value);
  double getThrottle();
  void setThrottle(double value);
}
