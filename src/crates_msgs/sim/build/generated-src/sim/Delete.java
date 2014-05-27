package sim;

public interface Delete extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Delete";
  static final java.lang.String _DEFINITION = "string model_name                 # name of the Gazebo Model to be deleted\n---\nbool success                      # return true if deletion is successful\nstring status_message             # comments if available\n";
}
