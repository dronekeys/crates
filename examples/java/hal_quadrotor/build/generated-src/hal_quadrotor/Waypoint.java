package hal_quadrotor;

public interface Waypoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/Waypoint";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 yaw\n---\n# RESULT\nbool    success\nstring  status";
}
