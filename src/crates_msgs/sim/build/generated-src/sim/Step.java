package sim;

public interface Step extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Step";
  static final java.lang.String _DEFINITION = "uint32 num_steps                  # number of steps\n---\nbool success                      # return true if spawn successful\nstring status_message             # comments if available\n";
}
