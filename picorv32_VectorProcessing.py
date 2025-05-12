#!/usr/bin/env python3

import os
from siliconcompiler import Chip
#import VectorProcessingV3
from siliconcompiler.targets import skywater130_demo

try:
    from . import VectorProcessingV3
except:  # noqa E722
    import VectorProcessingV3


def build_top():
    # Core settings.
    design = 'picorv32_top'
    target = skywater130_demo
    die_w = 1000
    die_h = 1000

    # Create Chip object.
    chip = Chip(design)

    # Set default Skywater130 PDK / standard cell lib / flow.
    chip.use(target)

    # Set design source files.)
    chip.register_source(name='picorv32',
                         path='git+https://github.com/YosysHQ/picorv32.git',
                         ref='c0acaebf0d50afc6e4d15ea9973b60f5f4d03c42')
    chip.input(os.path.join(os.path.dirname(__file__), f"{design}.v"))
    chip.input("picorv32.v", package='picorv32')

    # Optional: silence each task's output in the terminal.
    chip.set('option', 'quiet', True)

    # Set die outline and core area.
    margin = 10
    chip.set('constraint', 'outline', [(0, 0), (die_w, die_h)])
    chip.set('constraint', 'corearea', [(margin, margin),
                                        (die_w - margin, die_h - margin)])

    # Setup Vector Processing macro library.
    chip.use(VectorProcessingV3, stackup=chip.get('option', 'stackup'))
    chip.add('asic', 'macrolib', 'VectorProcessingV3')

    # SRAM pins are inside the macro boundary; no routing blockage padding is needed.
    chip.set('tool', 'openroad', 'task', 'route', 'var', 'grt_macro_extension', '0')
    # Disable CDL file generation until we can find a CDL file for the SRAM block.
    chip.set('tool', 'openroad', 'task', 'export', 'var', 'write_cdl', 'false')
    # Reduce placement density a bit to ease routing congestion and to speed up the route step.
    chip.set('tool', 'openroad', 'task', 'place', 'var', 'place_density', '0.5')

    # Place macro instance.
    chip.set('constraint', 'component', 'VectorProcessing', 'placement', (150, 150))
    chip.set('constraint', 'component', 'VectorProcessing', 'rotation', 'R180')

    # Set clock period, so that we won't need to provide an SDC constraints file.
    chip.clock('clk', period=25)

    # Run the build.
    chip.set('option', 'remote', True)
    chip.set('option', 'quiet', False)

    chip.run()

    # Print results.
    chip.summary()

    return chip


if __name__ == '__main__':
    build_top()