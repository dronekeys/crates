package sim;

public interface NoiseRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/NoiseRequest";
  static final java.lang.String _DEFINITION = "bool   enable                  \t  # Enable or disable this noise process\n";
  boolean getEnable();
  void setEnable(boolean value);
}
