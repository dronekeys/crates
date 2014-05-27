package sim;

public interface StepRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/StepRequest";
  static final java.lang.String _DEFINITION = "uint32 num_steps                  # number of steps\n";
  int getNumSteps();
  void setNumSteps(int value);
}
