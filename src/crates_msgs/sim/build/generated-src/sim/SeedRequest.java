package sim;

public interface SeedRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/SeedRequest";
  static final java.lang.String _DEFINITION = "uint32 seed                  \t  # random seed\n";
  int getSeed();
  void setSeed(int value);
}
