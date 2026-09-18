"""Send NGK tessellation data to OCP Viewer."""

import json

from ..core import _to_tcv_json


def to_tcv(obj, name=None, color="#e8b024", alpha=1.0):
    """Convert an NGK shape to viewer-quality TCV data."""
    return json.loads(_to_tcv_json(obj, name=name, color=color, alpha=alpha))


def show(obj, name=None, port=None, color="#e8b024", alpha=1.0, **viewer_config):
    """Send an NGK shape to an OCP Viewer session.

    Install the package with its OCP-viewer extra before using this adapter.
    """
    from ocp_vscode.comms import send_data

    config = {"reset_camera": "reset", "render_edges": True}
    config.update(viewer_config)
    payload = {
        "data": {"instances": [], "shapes": to_tcv(obj, name=name, color=color, alpha=alpha)},
        "type": "data",
        "config": config,
        "count": 1,
    }
    return send_data(payload, port=port)
