package sim;

public interface Seed extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Seed";
  static final java.lang.String _DEFINITION = "uint32 seed                  \t  # random seed\n---\nbool success                      # return true if spawn successful\nstring status_message             # comments if available\n";
}
