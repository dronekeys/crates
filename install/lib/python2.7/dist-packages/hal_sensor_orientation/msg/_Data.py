"""autogenerated by genpy from hal_sensor_orientation/Data.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Data(genpy.Message):
  _md5sum = "ba1b240e49880473c3351206f39f1453"
  _type = "hal_sensor_orientation/Data"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 t       # Time at which measurement was taken
float64 roll   	# Roll  X
float64 pitch  	# Pitch Y
float64 yaw    	# Yaw   Z
float64 p   	# Angvel X
float64 q  		# Angvel Y
float64 r   	# Angvel Z
"""
  __slots__ = ['t','roll','pitch','yaw','p','q','r']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       t,roll,pitch,yaw,p,q,r

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Data, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.t is None:
        self.t = 0.
      if self.roll is None:
        self.roll = 0.
      if self.pitch is None:
        self.pitch = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.p is None:
        self.p = 0.
      if self.q is None:
        self.q = 0.
      if self.r is None:
        self.r = 0.
    else:
      self.t = 0.
      self.roll = 0.
      self.pitch = 0.
      self.yaw = 0.
      self.p = 0.
      self.q = 0.
      self.r = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.t, _x.roll, _x.pitch, _x.yaw, _x.p, _x.q, _x.r))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.t, _x.roll, _x.pitch, _x.yaw, _x.p, _x.q, _x.r,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.t, _x.roll, _x.pitch, _x.yaw, _x.p, _x.q, _x.r))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 56
      (_x.t, _x.roll, _x.pitch, _x.yaw, _x.p, _x.q, _x.r,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7d = struct.Struct("<7d")
