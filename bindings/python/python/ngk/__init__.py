import json

from .geometry import *
from .modeling.edges import line
from .modeling.faces import rectangle as rectangle_face
from .modeling.profiles import rectangle as rectangle_profile
from .modeling.solids import block, cut, fuse, intersect
from .exchange.step import StepImport, read_step, step_from_string, step_to_string, write_step
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

