# The actual model to inherit from
from .biomechanical_model import BiomechanicalModel

# Some classes to define the BiomechanicalModel
from .axis import Axis
from .inertia_parameters import InertiaParameters
from .marker import Marker
from .contact import Contact
from .mesh import Mesh
from .protocols import Data, GenericDynamicModel
from .rotations import Rotations
from .range_of_motion import RangeOfMotion, Ranges
from .segment import Segment
from .segment_coordinate_system import SegmentCoordinateSystem
from .translations import Translations

# Add also the "Real" version of classes to create models from values
from .biomechanical_model_real import BiomechanicalModelReal
from .axis_real import AxisReal
from .marker_real import MarkerReal
from .contact_real import ContactReal
from .mesh_real import MeshReal
from .segment_real import SegmentReal
from .segment_coordinate_system_real import SegmentCoordinateSystemReal
from .inertia_parameters_real import InertiaParametersReal

# The accepted data formating
from .c3d_data import C3dData
