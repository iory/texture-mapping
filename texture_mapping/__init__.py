# flake8: noqa

import pkg_resources
import os.path as osp


__version__ = pkg_resources.get_distribution('texture-mapping').version


texture_mapping_executable = osp.join(
    osp.abspath(osp.dirname(__file__)), 'texture_mapping')
