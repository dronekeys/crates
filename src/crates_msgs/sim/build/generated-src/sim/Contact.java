package sim;

public interface Contact extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Contact";
  static final java.lang.String _DEFINITION = "string name1\nstring name2\n";
  java.lang.String getName1();
  void setName1(java.lang.String value);
  java.lang.String getName2();
  void setName2(java.lang.String value);
}
