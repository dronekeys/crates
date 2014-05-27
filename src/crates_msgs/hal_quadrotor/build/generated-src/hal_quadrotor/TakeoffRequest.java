package hal_quadrotor;

public interface TakeoffRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/TakeoffRequest";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 altitude\n";
  double getAltitude();
  void setAltitude(double value);
}
