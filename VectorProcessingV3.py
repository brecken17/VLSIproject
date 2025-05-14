from siliconcompiler import Library
import os

def setup(stackup=None):

	lib = Library('VectorProcessingV3', package='VectorProcessing', auto_enable=True)
	lib.register_source('VectorProcessing', path=__file__)
	lib.input('VectorProcessingV3.v')
	lib.add('option', 'idir', 'include')

	rootdir = os.path.dirname(__file__)
	lib.set('output', 'blackbox', 'verilog', os.path.join(rootdir, "VectorProcessingV3.bb.v"))
    	# Ensure this file gets uploaded to remote
    	#lib.set('output', 'blackbox', 'verilog', True, field='copy')

	return lib

