package hal_quadrotor;

public interface GetTruthResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/GetTruthResponse";
  static final java.lang.String _DEFINITION = "hal_quadrotor/State state";
  hal_quadrotor.State getState();
  void setState(hal_quadrotor.State value);
}
