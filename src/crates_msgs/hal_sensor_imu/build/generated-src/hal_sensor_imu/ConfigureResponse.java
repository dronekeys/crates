package hal_sensor_imu;

public interface ConfigureResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_sensor_imu/ConfigureResponse";
  static final java.lang.String _DEFINITION = "bool    success\nstring  status";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
}
