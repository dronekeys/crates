package hal_quadrotor;

public interface SetEstimateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/SetEstimateRequest";
  static final java.lang.String _DEFINITION = "hal_quadrotor/State state\n";
  hal_quadrotor.State getState();
  void setState(hal_quadrotor.State value);
}
