import sys
import types

import pytest

import ngk


def test_tcv_constructors_return_supported_wrappers():
    assert type(ngk.line((0, 0, 0), (1, 0, 0))).__name__ == "Edge"
    assert type(ngk.rectangle_profile(2, 3)).__name__ == "Profile"
    assert type(ngk.rectangle_face(2, 3)).__name__ == "Face"
    assert type(ngk.block(1, 2, 3)).__name__ == "Solid"


@pytest.mark.parametrize(
    "shape",
    [
        lambda: ngk.line((0, 0, 0), (1, 0, 0)),
        lambda: ngk.rectangle_profile(2, 3),
        lambda: ngk.rectangle_face(2, 3),
        lambda: ngk.block(1, 2, 3),
    ],
)
def test_to_tcv_returns_plain_dict_for_supported_wrappers(shape):
    data = ngk.to_tcv(shape(), name="part")

    assert isinstance(data, dict)
    assert data["version"] == 3
    assert data["name"] == "part"
    assert data["id"] == "/part"
    assert len(data["parts"]) == 1
    assert "shape" in data["parts"][0]


def test_to_tcv_rejects_unsupported_objects():
    with pytest.raises(TypeError, match="ngk.to_tcv supports"):
        ngk.to_tcv(object())


def test_show_sends_tcv_payload_through_ocp_vscode(monkeypatch):
    sent = {}

    def fake_send_data(payload, port=None):
        sent["payload"] = payload
        sent["port"] = port
        return "viewer"

    comms = types.SimpleNamespace(send_data=fake_send_data)
    fake_ocp = types.SimpleNamespace(comms=comms)
    monkeypatch.setitem(sys.modules, "ocp_vscode", fake_ocp)
    monkeypatch.setitem(sys.modules, "ocp_vscode.comms", comms)

    result = ngk.show(ngk.line((0, 0, 0), (1, 0, 0)), name="line", port=3940, grid=True)

    assert result == "viewer"
    assert sent["port"] == 3940
    assert sent["payload"]["type"] == "data"
    assert sent["payload"]["count"] == 1
    assert sent["payload"]["config"]["reset_camera"] == "reset"
    assert sent["payload"]["config"]["render_edges"] is True
    assert sent["payload"]["config"]["grid"] is True
    assert sent["payload"]["data"]["instances"] == []
    assert sent["payload"]["data"]["shapes"]["name"] == "line"

