"""Writing STEP from Python, checked against build123d where it is available.

The round trip through build123d is the test that matters: it is an
independent kernel reading what ngk wrote, so it catches a file that satisfies
every internal invariant and is still refused by a real reader. It is skipped
rather than failed where build123d is not installed, since the rest of these
do not need it.
"""

import os

import pytest

from ngk.exchange import step
from ngk.modeling import booleans, solids

try:
    import build123d as _b3d
except ImportError:  # pragma: no cover - depends on the environment
    _b3d = None

needs_build123d = pytest.mark.skipif(_b3d is None, reason="build123d is not installed")


def test_write_step_returns_a_path_that_exists():
    path = step.write_step(solids.block(10.0, 20.0, 30.0))

    assert os.path.isfile(path)
    assert path.endswith(".step")
    os.remove(path)


def test_write_step_honours_an_explicit_path(tmp_path):
    target = tmp_path / "block.step"

    written = step.write_step(solids.block(1.0, 1.0, 1.0), str(target))

    assert written == str(target)
    assert target.read_text().startswith("ISO-10303-21;")


def test_write_step_names_the_product():
    text = step.step_to_string(solids.block(1.0, 1.0, 1.0), name="WIDGET")

    assert "PRODUCT('WIDGET','WIDGET'" in text


def test_step_to_string_writes_no_file(tmp_path):
    before = set(os.listdir(tmp_path))

    text = step.step_to_string(solids.block(1.0, 1.0, 1.0))

    assert text.startswith("ISO-10303-21;")
    assert text.rstrip().endswith("END-ISO-10303-21;")
    assert set(os.listdir(tmp_path)) == before


def test_cylinder_wall_round_trips_through_ngk_step_text():
    result = booleans.cut(
        solids.block(8.0, 8.0, 8.0),
        solids.block(2.0, 2.0, 20.0),
    )

    restored = step.step_from_string(step.step_to_string(result))

    assert len(restored.solids) == 1


def test_writing_into_a_missing_directory_raises_oserror(tmp_path):
    missing = tmp_path / "no_such_directory" / "block.step"

    with pytest.raises(OSError):
        step.write_step(solids.block(1.0, 1.0, 1.0), str(missing))


@needs_build123d
def test_a_block_round_trips_through_build123d(tmp_path):
    path = str(tmp_path / "block.step")
    step.write_step(solids.block(10.0, 20.0, 30.0), path, name="BLOCK")

    solids = _b3d.import_step(path).solids()
    assert len(solids) == 1

    solid = solids[0]
    assert solid.is_valid
    assert solid.volume == pytest.approx(6000.0)
    assert solid.area == pytest.approx(2200.0)
    assert (len(solid.faces()), len(solid.edges()), len(solid.vertices())) == (6, 12, 8)

    size = solid.bounding_box().size
    assert (size.X, size.Y, size.Z) == pytest.approx((10.0, 20.0, 30.0))


def test_read_step_reads_what_write_step_wrote(tmp_path):
    path = str(tmp_path / "block.step")
    step.write_step(solids.block(10.0, 20.0, 30.0), path, name="BLOCK")

    result = step.read_step(path)

    assert len(result) == 1
    assert result.skipped == []
    solid = result.solids[0]
    assert (len(solid.faces()), len(solid.edges()), len(solid.vertices())) == (6, 12, 8)


def test_step_from_string_reads_no_file(tmp_path):
    before = set(os.listdir(tmp_path))

    text = step.step_to_string(solids.block(1.0, 2.0, 3.0))
    result = step.step_from_string(text)

    assert len(result.solids) == 1
    assert set(os.listdir(tmp_path)) == before


def test_reading_a_file_that_is_not_there_raises_oserror(tmp_path):
    with pytest.raises(OSError):
        step.read_step(str(tmp_path / "absent.step"))


def test_reading_malformed_text_raises_valueerror():
    with pytest.raises(ValueError) as raised:
        step.step_from_string("this is not a STEP file\n")

    assert "line 1" in str(raised.value)


@needs_build123d
def test_a_build123d_box_imports_into_ngk(tmp_path):
    # The direction the export tests cannot cover: another kernel's file read
    # into an ngk map. A box written by OpenCascade carries entities ngk never
    # writes — SURFACE_CURVE wrapping each line, PCURVEs we ignore, and plain
    # FACE_BOUNDs where we would write FACE_OUTER_BOUND — so this is the test
    # that the reader is lenient in the ways real files require.
    path = str(tmp_path / "b3d_box.step")
    _b3d.export_step(_b3d.Box(10, 20, 30), path)

    result = step.read_step(path)

    assert result.skipped == []
    assert len(result.solids) == 1
    solid = result.solids[0]
    assert (len(solid.faces()), len(solid.edges()), len(solid.vertices())) == (6, 12, 8)


@needs_build123d
def test_a_build123d_solid_survives_a_trip_through_ngk(tmp_path):
    # All the way round: build123d writes, ngk reads and writes again, and
    # build123d reads the result back as the same solid. Each kernel only ever
    # checks its own output otherwise.
    first = str(tmp_path / "in.step")
    second = str(tmp_path / "out.step")
    _b3d.export_step(_b3d.Box(10, 20, 30), first)

    result = step.read_step(first)
    step.write_step(result.solids[0], second, name="ROUND_TRIP")

    solid = _b3d.import_step(second).solids()[0]
    assert solid.is_valid
    assert solid.volume == pytest.approx(6000.0)
    assert solid.area == pytest.approx(2200.0)
    size = solid.bounding_box().size
    assert (size.X, size.Y, size.Z) == pytest.approx((10.0, 20.0, 30.0))
