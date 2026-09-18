from pathlib import Path
from ngk.exchange import step
from ngk.modeling import solids
from ngk.viz import debug
p = Path(r"D:\Projets\ngk\tests\exchange\foreign\files")

files = p.glob("*.step")



solid = solids.block(1.0, 2.0, 3.0)
solid = solids.cylinder(1.0, 2.0)
# debug.show(solid, name="solid")
for file in files:
    print(file)
    result = step.read_step(file)
    c = result.solids[0]
    debug.show(c, name=file.stem)
        # break
