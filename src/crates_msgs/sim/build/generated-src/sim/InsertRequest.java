package sim;

public interface InsertRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/InsertRequest";
  static final java.lang.String _DEFINITION = "string model_name                 # name of the model to be spawn\nstring model_type                 # spawn robot and all ROS interfaces under this namespace\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
  java.lang.String getModelType();
  void setModelType(java.lang.String value);
}
