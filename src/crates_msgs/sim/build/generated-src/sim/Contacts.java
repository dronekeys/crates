package sim;

public interface Contacts extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sim/Contacts";
  static final java.lang.String _DEFINITION = "sim/Contact[] contacts\n";
  java.util.List<sim.Contact> getContacts();
  void setContacts(java.util.List<sim.Contact> value);
}
