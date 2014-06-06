package hal_quadrotor;

public interface AnglesHeight extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/AnglesHeight";
  static final java.lang.String _DEFINITION = "# INSTRUCTION\nfloat64 roll\nfloat64 pitch\nfloat64 yaw\nfloat64 z\n---\n# RESULT\nbool    success\nstring  status\n";
}
