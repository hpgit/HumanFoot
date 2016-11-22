from __future__ import print_function
from __future__ import absolute_import
import sys

from . import pydart2_api as papi
from . import world
from . import constraints
from . import utils
from .world import World
from .skel_vector import SkelVector

# for type hinting
from .joint import Joint, BallJoint, EulerJoint, FreeJoint, PlanarJoint, PrismaticJoint, RevoluteJoint
from .joint import ScrewJoint, TranslationalJoint, UniversalJoint, WeldJoint
from .dof import Dof
from .bodynode import BodyNode
from .skeleton import Skeleton
from .shapenode import ShapeNode
from .marker import Marker
from .contact import Contact
from .constraints import BallJointConstraint
from .collision_result import CollisionResult
from .shape import Shape, SoftMeshShape, BoxShape, CylinderShape, EllipsoidShape, LineSegmentShape, PlaneShape, MeshShape

assert utils
assert World
assert SkelVector

assert world
assert constraints

# import os.path

try:
    from . import gui
    assert gui
except Exception:
    e = sys.exc_info()[1]
    print("-" * 40)
    print("Error while importing pydart2.gui")
    print(e)
    print("-" * 40)

__version__ = (0, 5, 0)


def init(verbose=True):
    papi.init(verbose)
