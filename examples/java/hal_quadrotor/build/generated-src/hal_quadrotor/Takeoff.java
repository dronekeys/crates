package hal_quadrotor;

public interface Takeoff extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/Takeoff";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 altitude\n---\n# RESULT\nbool    success\nstring  status";
}
