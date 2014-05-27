package hal_quadrotor;

public interface AnglesHeightResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/AnglesHeightResponse";
  static final java.lang.String _DEFINITION = "# RESULT\nbool    success\nstring  status";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}