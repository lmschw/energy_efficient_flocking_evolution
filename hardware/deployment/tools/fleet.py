"""Reads the fleet settings (fleet.conf, or the file in $FLEET_CONF) for the Python tools: PI_USER, KEY (ssh key)
and IPS (robot number -> Pi IP address)."""
import os

_path = os.environ.get("FLEET_CONF") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "fleet.conf")
_conf = {}
for _line in open(_path):
    _line = _line.split("#", 1)[0].strip()
    if "=" in _line:
        _k, _v = _line.split("=", 1)
        _conf[_k.strip()] = os.path.expanduser(os.path.expandvars(_v.strip()))
PI_USER = _conf["PI_USER"]
KEY = _conf["PI_KEY"]
IPS = {n: _conf[f"ROBOT_{n}"] for n in range(1, 8)}
