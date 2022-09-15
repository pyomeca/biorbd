# The actual model to inherit from
from .biomechanical_model import BiomechanicalModel

# Some classes to define the BiomechanicalModel
from .axis import Axis
from .marker import Marker
from .mesh import Mesh
from .protocols import Data, GenericDynamicModel
from .segment import Segment
from .segment_coordinate_system import SegmentCoordinateSystem
from .inertia_parameters import InertiaParameters

# Add also the "Real" version of classes to create models from values
from .biomechanical_model_real import BiomechanicalModelReal
from .axis_real import AxisReal
from .marker_real import MarkerReal
from .mesh_real import MeshReal
from .segment_real import SegmentReal
from .segment_coordinate_system_real import SegmentCoordinateSystemReal
from .inertia_parameters_real import InertiaParametersReal

# The accepted data formating
from .c3d_data import C3dData

