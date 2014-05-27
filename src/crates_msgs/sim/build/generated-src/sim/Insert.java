package sim;

public interface Insert extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Insert";
  static final java.lang.String _DEFINITION = "string model_name                 # name of the model to be spawn\nstring model_type                 # spawn robot and all ROS interfaces under this namespace\n---\nbool success                      # return true if spawn successful\nstring status_message             # comments if available\n";
}
