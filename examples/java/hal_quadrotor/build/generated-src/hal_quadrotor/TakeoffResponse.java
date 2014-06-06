package hal_quadrotor;

public interface TakeoffResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/TakeoffResponse";
  static final java.lang.String _DEFINITION = "# RESULT\nbool    success\nstring  status";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}
