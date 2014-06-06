package hal_quadrotor;

public interface SetControlRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/SetControlRequest";
  static final java.lang.String _DEFINITION = "hal_quadrotor/Control control\n";
  hal_quadrotor.Control getControl();
  void setControl(hal_quadrotor.Control value);
}
