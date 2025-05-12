import siliconcompiler
import os


def setup(stackup=None):
    # Core values.
    design = 'VectorProcessingV3'

    if stackup is None:
        raise RuntimeError('stackup cannot be None')

    # Create library Chip object.
    lib = siliconcompiler.Library(design)
    lib.register_source('vlsiVectorProcessing',
                        'git+https://github.com/brecken17/VLSIproject')
    lib.set('output', stackup, 'gds',
            'VectorProcessingV3/VectorProcessingV3.gds',
            package='vlsiVectorProcessing')
    lib.set('output', stackup, 'lef',
            'VectorProcessingV3/VectorProcessingV3.lef',
            package='vlsiVectorProcessing')

    rootdir = os.path.dirname(__file__)
    lib.set('output', 'blackbox', 'verilog', os.path.join(rootdir, "VectorProcessingV3.bb.v"))
    # Ensure this file gets uploaded to remote
    lib.set('output', 'blackbox', 'verilog', True, field='copy')

    return lib