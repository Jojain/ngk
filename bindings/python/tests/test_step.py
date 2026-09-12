"""Writing STEP from Python, checked against build123d where it is available.

The round trip through build123d is the test that matters: it is an
independent kernel reading what ngk wrote, so it catches a file that satisfies
every internal invariant and is still refused by a real reader. It is skipped
rather than failed where build123d is not installed, since the rest of these
do not need it.
"""

import os

import pytest

import ngk

try:
    import build123d as _b3d
except ImportError:  # pragma: no cover - depends on the environment
    _b3d = None

needs_build123d = pytest.mark.skipif(_b3d is None, reason="build123d is not installed")


def test_write_step_returns_a_path_that_exists():
    path = ngk.write_step(ngk.block(10.0, 20.0, 30.0))

    assert os.path.isfile(path)
    assert path.endswith(".step")
    os.remove(path)


def test_write_step_honours_an_explicit_path(tmp_path):
    target = tmp_path / "block.step"

    written = ngk.write_step(ngk.block(1.0, 1.0, 1.0), str(target))

    assert written == str(target)
    assert target.read_text().startswith("ISO-10303-21;")


def test_write_step_names_the_product():
    text = ngk.step_to_string(ngk.block(1.0, 1.0, 1.0), name="WIDGET")

    assert "PRODUCT('WIDGET','WIDGET'" in text


def test_step_to_string_writes_no_file(tmp_path):
    before = set(os.listdir(tmp_path))

    text = ngk.step_to_string(ngk.block(1.0, 1.0, 1.0))

    assert text.startswith("ISO-10303-21;")
    assert text.rstrip().endswith("END-ISO-10303-21;")
    assert set(os.listdir(tmp_path)) == before


def test_geometry_that_cannot_be_written_raises_rather_than_writing_garbage():
    # Curved supports arrive in stage 4 of the STEP interop plan. Until then a
    # cylinder must refuse by name, not produce a file that is quietly wrong.
    with pytest.raises(ValueError) as raised:
        ngk.step_to_string(ngk.cut(ngk.block(8.0, 8.0, 8.0), ngk.block(2.0, 2.0, 20.0)))

    assert "not written yet" in str(raised.value)


def test_writing_into_a_missing_directory_raises_oserror(tmp_path):
    missing = tmp_path / "no_such_directory" / "block.step"

    with pytest.raises(OSError):
        ngk.write_step(ngk.block(1.0, 1.0, 1.0), str(missing))


@needs_build123d
def test_a_block_round_trips_through_build123d(tmp_path):
    path = str(tmp_path / "block.step")
    ngk.write_step(ngk.block(10.0, 20.0, 30.0), path, name="BLOCK")

    solids = _b3d.import_step(path).solids()
    assert len(solids) == 1

    solid = solids[0]
    assert solid.is_valid
    assert solid.volume == pytest.approx(6000.0)
    assert solid.area == pytest.approx(2200.0)
    assert (len(solid.faces()), len(solid.edges()), len(solid.vertices())) == (6, 12, 8)

    size = solid.bounding_box().size
    assert (size.X, size.Y, size.Z) == pytest.approx((10.0, 20.0, 30.0))
