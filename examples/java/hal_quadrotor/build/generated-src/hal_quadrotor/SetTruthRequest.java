package hal_quadrotor;

public interface SetTruthRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/SetTruthRequest";
  static final java.lang.String _DEFINITION = "hal_quadrotor/State state\n";
  hal_quadrotor.State getState();
  void setState(hal_quadrotor.State value);
}
