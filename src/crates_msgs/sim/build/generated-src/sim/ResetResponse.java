package sim;

public interface ResetResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/ResetResponse";
  static final java.lang.String _DEFINITION = "bool success                      # return true if spawn successful\nstring status_message             # comments if available";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}