"""Send NGK objects to a separately running NGK debug viewer."""

import json
import urllib.error
import urllib.request

_ENDPOINT = "/__ngk_debug/dumps"
_ENTITY_KINDS = frozenset({"vertex", "edge", "profile", "face", "sheet", "solid"})


def show(*objects, name=None, host="127.0.0.1", port=3941, timeout=1.0):
    """Send topology objects to the local NGK debug viewer.

    The wheel supplies this transport only. Start the NGK debug viewer
    separately before calling it.
    """
    if not objects:
        raise TypeError("ngk.viz.debug.show requires at least one object")
    nodes = [_node(value, f"{type(value).__name__.lower()} {index}") for index, value in enumerate(objects)]
    payload = {
        "kind": "ngk.debug.v4",
        "name": str(name or nodes[0]["name"]),
        "nodes": nodes,
    }
    _send(
        urllib.request.Request(
            f"http://{host}:{port}{_ENDPOINT}",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        ),
        host,
        port,
        timeout,
    )


def clear(host="127.0.0.1", port=3941, timeout=1.0):
    """Clear objects held by the local NGK debug viewer."""
    _send(urllib.request.Request(f"http://{host}:{port}{_ENDPOINT}", method="DELETE"), host, port, timeout)


def _node(value, name):
    obj = _debug_object(value)
    if obj is not None:
        return {"name": name, "object": obj}
    if isinstance(value, (str, bytes)) or not hasattr(value, "__iter__"):
        raise TypeError(f"ngk.viz.debug.show does not support {type(value).__name__}")
    return {"name": name, "children": [_node(item, f"{name}[{index}]") for index, item in enumerate(value)]}


def _debug_object(obj):
    kind = type(obj).__name__.lower()
    if kind == "model":
        return {"kind": "model", "serialized": obj.serialize()}
    if kind in _ENTITY_KINDS:
        entity = obj.isolated()
        return {"kind": kind, "primaryDart": entity.dart_id, "serialized": entity.model.serialize()}
    return None


def _send(request, host, port, timeout):
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f"debug viewer rejected request: HTTP {response.status}")
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"debug viewer rejected request: HTTP {error.code}") from error
    except urllib.error.URLError as error:
        raise ConnectionError(f"could not connect to debug viewer at {host}:{port}") from error
