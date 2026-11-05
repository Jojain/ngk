"""Binary STL through the public Python exchange API."""

import struct

import pytest

from ngk.exchange import stl
from ngk.modeling import solids


def test_block_bytes_and_file_agree(tmp_path):
    solid = solids.block(2.0, 3.0, 4.0)
    data = stl.stl_to_bytes(solid)
    assert isinstance(data, bytes)
    assert struct.unpack_from("<I", data, 80) == (12,)
    assert len(data) == 684
    target = tmp_path / "block.stl"
    assert stl.write_stl(solid, str(target)) == str(target)
    assert target.read_bytes() == data


def test_invalid_sampling_preserves_existing_file(tmp_path):
    target = tmp_path / "existing.stl"
    target.write_bytes(b"preserved")
    with pytest.raises(ValueError, match="sampling"):
        stl.write_stl(solids.block(1.0, 1.0, 1.0), str(target), segments=0)
    assert target.read_bytes() == b"preserved"
