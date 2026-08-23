import json

from ._ngk import *
from ._ngk import _to_tcv_json


def to_tcv(obj, name=None, color="#e8b024", alpha=1.0):
    return json.loads(_to_tcv_json(obj, name=name, color=color, alpha=alpha))


def show(obj, name=None, port=None, color="#e8b024", alpha=1.0, **viewer_config):
    from ocp_vscode.comms import send_data

    config = {"reset_camera": "reset", "render_edges": True}
    config.update(viewer_config)
    shapes = to_tcv(obj, name=name, color=color, alpha=alpha)
    payload = {
        "data": {"instances": [], "shapes": shapes},
        "type": "data",
        "config": config,
        "count": 1,
    }
    return send_data(payload, port=port)

