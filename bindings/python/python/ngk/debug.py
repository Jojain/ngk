"""Send Python-side NGK topology objects to the local NGK debug viewer."""

import ast
import inspect
import json
import linecache
import urllib.error
import urllib.request

_ENDPOINT = "/__ngk_debug/dumps"
_ENTITY_KINDS = frozenset({"vertex", "edge", "profile", "face", "sheet", "solid"})


def show(*objects, name=None, host="127.0.0.1", port=3941, timeout=1.0):
    """Send shapes to the local NGK debug viewer, one tree node per argument.

    Each argument becomes a node the viewer shows and hides on its own, and an
    iterable argument becomes a group holding one child per item — so
    ``show(part, part.faces())`` sends the solid and every one of its faces as
    two independently togglable branches.

    Nodes are named after the expressions written at the call site, which is
    why the example above labels them ``part`` and ``part.faces()``. A call the
    source cannot be read back from falls back to the value's type. ``name``
    titles the dump itself, defaulting to the first node's name.
    """
    if not objects:
        raise TypeError("ngk.debug.show requires at least one object")
    names = _node_names(objects, inspect.currentframe().f_back)
    payload = {
        "kind": "ngk.debug.v4",
        "name": _clean_name(name or names[0]),
        "nodes": [_node(value, node_name) for value, node_name in zip(objects, names)],
    }
    request = urllib.request.Request(f"http://{host}:{port}{_ENDPOINT}", data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"}, method="POST")
    _send(request, host, port, timeout)


def clear(host="127.0.0.1", port=3941, timeout=1.0):
    """Clear objects held by the local NGK debug viewer."""
    _send(urllib.request.Request(f"http://{host}:{port}{_ENDPOINT}", method="DELETE"), host, port, timeout)


def _node(value, name):
    """One viewer tree node: a leaf for a shape, a group for an iterable."""
    obj = _debug_object(value)
    if obj is not None:
        return {"name": name, "object": obj}
    items = _items(value)
    if items is None:
        raise TypeError(f"ngk.debug.show does not support {type(value).__name__}; pass a Model, a topology entity, or an iterable of those")
    return {"name": name, "children": [_node(item, f"{name}[{index}]") for index, item in enumerate(items)]}


def _debug_object(obj):
    """The transported form of one shape, or None if it is not one.

    An entity travels isolated, so a node carries the face or edge it names and
    not the whole model that face was read from — which is what lets the viewer
    hide one of them at a time.
    """
    kind = type(obj).__name__.lower()
    if kind == "model":
        return {"kind": "model", "serialized": obj.serialize()}
    if kind in _ENTITY_KINDS:
        entity = obj.isolated()
        return {"kind": kind, "primaryDart": entity.dart_id, "serialized": entity.model.serialize()}
    return None


def _items(value):
    """The items of an iterable argument, or None if it is not one."""
    if isinstance(value, (str, bytes)) or not hasattr(value, "__iter__"):
        return None
    return list(value)


def _node_names(values, frame):
    """A name per argument, preferring the expression the caller wrote."""
    fallback = [type(value).__name__.lower() for value in values]
    if len(values) > 1:
        fallback = [f"{name} {index}" for index, name in enumerate(fallback)]
    sources = _call_argument_sources(frame)
    if sources is None or len(sources) != len(values):
        return fallback
    return [source or default for source, default in zip(sources, fallback)]


def _call_argument_sources(frame):
    """Positional argument expressions of the call `frame` is executing.

    Returns None when the call site cannot be recovered — an interactive
    prompt, a frame with no source on disk, a call made through ``*args`` —
    and the caller then names the nodes after their types instead.
    """
    positions = getattr(inspect.getframeinfo(frame), "positions", None) if frame is not None else None
    if positions is None or positions.lineno is None or positions.col_offset is None:
        return None
    lines = linecache.getlines(frame.f_code.co_filename, frame.f_globals)
    if len(lines) < positions.end_lineno:
        return None
    # Column offsets count UTF-8 bytes, so the span is cut in bytes, end first
    # because a one-line call has both cuts landing on the same string.
    segment = [line.encode() for line in lines[positions.lineno - 1 : positions.end_lineno]]
    segment[-1] = segment[-1][: positions.end_col_offset]
    segment[0] = segment[0][positions.col_offset :]
    try:
        call = ast.parse(b"".join(segment).decode(), mode="eval").body
    except (SyntaxError, UnicodeDecodeError, ValueError):
        return None
    if not isinstance(call, ast.Call) or any(isinstance(arg, ast.Starred) for arg in call.args):
        return None
    return [ast.unparse(arg) for arg in call.args]


def _clean_name(name):
    clean = str(name).replace("/", "_").replace("\\", "_")
    return clean if clean.strip() else "shape"


def _send(request, host, port, timeout):
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f"debug viewer rejected request: HTTP {response.status}")
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"debug viewer rejected request: HTTP {error.code}") from error
    except urllib.error.URLError as error:
        raise ConnectionError(f"could not connect to debug viewer at {host}:{port}") from error
