import ngk
from pathlib import Path
p = Path(r"D:\Projets\ngk\tests\exchange\foreign\files")

files = p.glob("*.step")



solid = ngk.block(1.0, 2.0, 3.0)
solid = ngk.cylinder(1.0, 2.0)
# ngk.debug.show(solid, name="solid")
for file in files:
    print(file)
    step = ngk.exchange.step.read_step(file)
    c = step.solids[0]
    ngk.debug.show(c, name=file.stem)
        # break
