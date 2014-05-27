package sim;

public interface Noise extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Noise";
  static final java.lang.String _DEFINITION = "bool   enable                  \t  # Enable or disable this noise process\n---\nbool   success                    # return true if spawn successful\nstring status_message             # comments if available\n";
}
