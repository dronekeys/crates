package hal_quadrotor;

public interface GetControlResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/GetControlResponse";
  static final java.lang.String _DEFINITION = "hal_quadrotor/Control control";
  hal_quadrotor.Control getControl();
  void setControl(hal_quadrotor.Control value);
}
