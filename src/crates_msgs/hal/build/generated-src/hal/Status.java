package hal;

public interface Status extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal/Status";
  static final java.lang.String _DEFINITION = "Header          header    # Time\nuint8           status    # Status (0 = normal, 1... warning, error, fatal)\nstring          message   # Message string\ntime            start     # Time since activated";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getStatus();
  void setStatus(byte value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  org.ros.message.Time getStart();
  void setStart(org.ros.message.Time value);
}
