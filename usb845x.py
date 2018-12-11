#!/usr/bin/env python2
"""  wrapper for Ni845x.dll.
minimal functions implemented to get i2c running.

"""

import ctypes
import json
from ctypes import byref, c_void_p
import numpy as np

# pylint: disable=no-member,invalid-name

ni8451_dll = ctypes.WinDLL("./Ni845x.dll")

with open("nidefs.json",'rb') as jdefs:
  nidefs = json.load(jdefs)
with open("errors.json",'rb') as jerrs:
  nierrors = { int(k):v for k,v in json.load(jerrs).items()}


#define some NI types
NiHandle_p = ctypes.c_void_p
if ctypes.sizeof(ctypes.c_void_p) == 8: #64bit
  NiHandle = ctypes.c_uint64
else:
  NiHandle = ctypes.c_uint32

#list of ni API functions (incomplete) and their inputs.
_nifns_ret_t = ctypes.c_int32

#functions that generate a handle
_ni_init_fns = {
    "ni845xFindDevice":[ctypes.c_char_p,NiHandle_p,ctypes.c_void_p],
    "ni845xI2cConfigurationOpen":[NiHandle_p],
    "ni845xOpen":[ctypes.c_char_p,NiHandle_p]
}

_ni_i2c_data_fns = {
    #devh confh writesize writedata(uint8p) readsize(uint32p) readdata(uint8p)
    "ni845xSpiWriteRead": [NiHandle,NiHandle,ctypes.c_uint32,c_void_p,c_void_p,c_void_p],
    "ni845xI2cWrite": [NiHandle,NiHandle,ctypes.c_uint32,c_void_p]
}

#general functions that use an ni device handle
_ni_gen_fns = {
    "ni845xI2cSetPullupEnable":[NiHandle,ctypes.c_uint8],
    "ni845xSetIoVoltageLevel":[NiHandle,ctypes.c_uint8],
    "ni845xClose":[NiHandle],
    "ni845xDioReadLine":[NiHandle,ctypes.c_uint8,ctypes.c_uint8,ctypes.POINTER(ctypes.c_int32)], #handle, port no, line no, readval {0,1}
    "ni845xDioReadPort":[NiHandle,ctypes.c_uint8,ctypes.POINTER(ctypes.c_int8)], #handle, port no, readdata (all 8 bits)
    "ni845xDioSetPortLineDirectionMap":[NiHandle,ctypes.c_uint8,ctypes.c_uint8],
    "ni845xDioSetDriverType":[NiHandle,ctypes.c_uint8,ctypes.c_uint8],
    "ni845xDioWriteLine":[NiHandle,ctypes.c_uint8,ctypes.c_uint8,ctypes.c_int32], #handle, port no, line no, write val {0,1}
    "ni845xDioWritePort":[NiHandle,ctypes.c_uint8,ctypes.c_int8] #handle, port no, write data (all 8 bits)
}

#functions for i2c config use a device handle
_ni_i2c_conf_fns = {
    "ni845xI2cConfigurationSetAddressSize":[NiHandle,ctypes.c_int32],
    "ni845xI2cConfigurationSetAddress":[NiHandle,ctypes.c_uint16],
    "ni845xI2cConfigurationSetClockRate":[NiHandle,ctypes.c_uint16],
    "ni845xI2cConfigurationClose":[NiHandle]
}

_ni_all_fns = _ni_init_fns.copy()
_ni_all_fns.update(_ni_i2c_data_fns)
_ni_all_fns.update(_ni_i2c_conf_fns)
_ni_all_fns.update(_ni_gen_fns)


class NiI2cCh(object):
  def __init__(self,ni8451_dev,addr,freq=100):
    """
      ni8451_dev: must be an instance of NI8451 with an open device
      addr (int): i2c address to open
      freq (int): frequency (in khz), default = 100khz
    """
    if not ni8451_dev.is_open():
      raise AttributeError("device must be open")
    if addr < 0 or addr > 127:
      raise AttributeError("invalid address")
    self.i2ch = NiHandle()

    self.nidev = ni8451_dev
    self.I2cConfigurationOpen()
    self.I2cConfigurationSetAddressSize(0)
    self.I2cConfigurationSetAddress(addr)
    self.I2cConfigurationSetClockRate(freq) #100khz

  def I2cConfigurationOpen(self):
    errck( ni8451_dll.ni845xI2cConfigurationOpen(byref(self.i2ch)) )

  def I2cWriteRead(self,outdata,read_len):
    indat = np.zeros(read_len,dtype=np.uint8)
    nread = ctypes.c_uint32(0)
    if outdata.dtype == np.uint8 or outdata.dtype == np.int8:
      _od = outdata.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    else:
      raise TypeError("must be numpy array of np.uint8 or np.int8")
    errck (ni8451_dll.ni845xI2cWriteRead( self.nidev.devh,
                                          self.i2ch,
                                          len(outdata),
                                          _od,
                                          read_len,byref(nread),
                                          indat.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
                                        ))
    if nread.value != read_len:
      raise Exception("expected {} bytes, got {} bytes".format(read_len,nread.value))
    return indat

  def I2cWrite(self,outdata):
    if outdata.dtype == np.uint8 or outdata.dtype == np.int8:
      _od = outdata.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    else:
      raise TypeError("must be numpy array of np.uint8 or np.int8")
    errck (ni8451_dll.ni845xI2cWrite(self.nidev.devh,self.i2ch,len(outdata),_od))

class NI8451(object):
  def __init__(self):
    self.devh = NiHandle(0)

  def Open(self,dev_name):
    ni8451_dll.ni845xOpen(dev_name,byref(self.devh))

  #manually generated methods
  def FindFirstDevice(self):
    first_dev=ctypes.create_string_buffer(260)
    n_devs = ctypes.c_uint32(0)
    _h = NiHandle()
    ni8451_dll.ni845xFindDevice(first_dev,byref(_h),byref(n_devs))
    return first_dev.value

  def is_open(self):
    return self.devh.value!=0


def errck(x):
  """error checking function.
       checks NI error status and raises
       exception on error or warning
  """
  if x != 0:
    try:
      err_str = nierrors[x]
    except KeyError:
      err_str = "unknown error."
    raise Exception("NI error {}: {}".format(x,err_str))

def gen_i2c_conf_method(mname,cargs):
  """ Generates a call method for Ni function mname. """
  def _temp(self,*argv):
    errck( getattr(ni8451_dll,"ni845x"+mname)(self.i2ch,*argv) )
  return _temp

def gen_general_dev_method(mname,cargs):
  """ Generates a call method for Ni function mname. """
  def _temp(self,*argv):
    errck (getattr(ni8451_dll,"ni845x"+mname)(self.devh,*argv))
  return _temp

## setup arg types
for k, v in _ni_all_fns.iteritems():
  _fp = getattr(ni8451_dll,k)
  _fp.argtypes = v
  _fp.restype = _nifns_ret_t

#add i2c config methods to NiI2cCh class
for k,v in _ni_i2c_conf_fns.iteritems():
  fnname = k.replace("ni845x","",1)
  #setup methods with handles taken care of
  _meth = gen_i2c_conf_method(fnname,v)
  setattr(NiI2cCh,fnname,_meth)

for k,v in _ni_gen_fns.iteritems():
  fnname = k.replace("ni845x","",1)
  #setup methods with handles taken care of
  _meth = gen_general_dev_method(fnname,v)
  setattr(NI8451,fnname,_meth)

def __main():
  from time import sleep
  nid = NI8451()
  dev_name = nid.FindFirstDevice()
  led_addr_offset = 0x20 #=port 0 addr
  led_pin = 21
  led_pin_addr = led_pin+led_addr_offset
  try:
    nid.Open(dev_name)
    nid.SetIoVoltageLevel(nidefs["kNi845x33Volts"])
    nid.DioSetPortLineDirectionMap(0,0x01)
    nid.DioWritePort(0,0x01)
    #nid.I2cSetPullupEnable(1)
    iicd = NiI2cCh(nid,0x44)

    #iicd.I2cWrite(np.array([0x0B,0xA8],dtype=np.uint8)) #P12 LED OUT
    iicd.I2cWrite(np.array([0x0D,0xA2],dtype=np.uint8)) #P21 LED OUT
    iicd.I2cWrite(np.array([0x02,0x05],dtype=np.uint8)) #global current level 0x05=9mA
    iicd.I2cWrite(np.array([0x04,0x01],dtype=np.uint8)) #shutdown off
    while 1:
      iicd.I2cWrite(np.array([led_pin_addr,0x01],dtype=np.uint8)) #P21 LED ON
      sleep(1)
      iicd.I2cWrite(np.array([led_pin_addr,0x00],dtype=np.uint8)) #P21 LED OFF
      sleep(1)
    data = iicd.I2cWriteRead(np.array([0x0B],dtype=np.uint8),1)
    print "{:#08b}".format(data[0])


  finally:
    nid.Close()

if __name__ == "__main__":
  __main()
