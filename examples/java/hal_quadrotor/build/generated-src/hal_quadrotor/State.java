package hal_quadrotor;

public interface State extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "hal_quadrotor/State";
  static final java.lang.String _DEFINITION = "float64 t\t    \t# Time stamp\nfloat64 x\t    \t# n-frame X position (X == +East)\nfloat64 y\t   \t\t# n-frame Y position (Y == +North)\nfloat64 z\t    \t# n-frame Z position (Z == +Up)\nfloat64 roll\t    # n-frame roll (anti-clockwise about X)\nfloat64 pitch\t    # n-frame pitch (anti-clockwise about Y)\nfloat64 yaw\t    \t# n-frame yaw (anti-clockwise about Z)\nfloat64 u\t    \t# b-frame X velocity\nfloat64 v\t    \t# b-frame Y velocity\nfloat64 w\t    \t# b-frame Z velocity\nfloat64 p\t    \t# b-frame roll angular velocity\nfloat64 q\t    \t# b-frame pitch angular velocity\nfloat64 r\t    \t# b-frame yaw angular velocity\nfloat64 thrust\t    # Current thrust force\nfloat64 remaining\t# Flight time remaining";
  double getT();
  void setT(double value);
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getRoll();
  void setRoll(double value);
  double getPitch();
  void setPitch(double value);
  double getYaw();
  void setYaw(double value);
  double getU();
  void setU(double value);
  double getV();
  void setV(double value);
  double getW();
  void setW(double value);
  double getP();
  void setP(double value);
  double getQ();
  void setQ(double value);
  double getR();
  void setR(double value);
  double getThrust();
  void setThrust(double value);
  double getRemaining();
  void setRemaining(double value);
}
