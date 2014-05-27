package sim;

public interface Pause extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Pause";
  static final java.lang.String _DEFINITION = "---\nbool success                      # return true if spawn successful\nstring status_message             # comments if available\n";
}
