package hal_quadrotor;

public interface Velocity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/Velocity";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 u\nfloat64 v\nfloat64 w\nfloat64 yaw\n---\n# RESULT\nbool    success\nstring  status";
}
