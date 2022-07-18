# pyxdl

An open implementation of Verifone's XDL protocol, as used by POS devices like the VX820 and the E355, and probably many more.

This file can be used as a stand-alone tool or as a library.

## Stand-alone

Example, sending a file and setting a config variable:

```sh
python xdl.py -d /dev/ttyUSB0 -c '*GO=HELLO.BIN' HELLO.BIN
```

## Library

Example:

```python
from xdl import XDL

some_file = "HELLO.BIN"

xdl = XDL(port="/dev/ttyUSB0")
xdl.connect()
xdl.set_config_var("*GO", some_file)
xdl.send_file(some_file)
xdl.stop()
```
