"""Send Python-side NGK topology objects to the local NGK debug viewer."""

import json
import urllib.error
import urllib.request

_ENDPOINT = "/__ngk_debug/dumps"


def _debug_object(obj):
    kind = type(obj).__name__.lower()
    if kind == "gmap":
        return {"kind": "gmap", "serialized": obj.serialize()}
    if kind in {"vertex", "edge", "profile", "face", "sheet", "solid"}:
        return {"kind": kind, "primaryDart": obj.dart_id, "serialized": obj.gmap.serialize()}
    raise TypeError("ngk.debug.show supports GMap and topology cell objects")


def show(obj, name=None, host="127.0.0.1", port=3941, timeout=1.0):
    """Send topology objects to the local NGK debug viewer."""
    objects = obj if isinstance(obj, (list, tuple)) else [obj]
    payload = {"kind": "ngk.debug.v3", "name": (name or "shape").replace("/", "_").replace("\\", "_") or "shape", "objects": [_debug_object(item) for item in objects]}
    request = urllib.request.Request(f"http://{host}:{port}{_ENDPOINT}", data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"}, method="POST")
    _send(request, host, port, timeout)


def clear(host="127.0.0.1", port=3941, timeout=1.0):
    """Clear objects held by the local NGK debug viewer."""
    _send(urllib.request.Request(f"http://{host}:{port}{_ENDPOINT}", method="DELETE"), host, port, timeout)


def _send(request, host, port, timeout):
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f"debug viewer rejected request: HTTP {response.status}")
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"debug viewer rejected request: HTTP {error.code}") from error
    except urllib.error.URLError as error:
        raise ConnectionError(f"could not connect to debug viewer at {host}:{port}") from error
