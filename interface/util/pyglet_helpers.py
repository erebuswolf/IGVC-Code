__author__="ericperk"
__date__ ="$Sep 21, 2009 1:26:39 PM$"

from pyglet.gl import GL_LINES

from stretching_pyglets_wings.data import Color
from stretching_pyglets_wings.primitive import Primitive
from stretching_pyglets_wings.shape import Shape

harlie_verts = [
    (0.5,0.0),
    (-0.5,0.0),
    (0.0,1.0),
]

x_axis_verts = [
    (0.0,0.0),
    (10.0,0.0),
]

y_axis_verts = [
    (0.0,0.0),
    (0.0,10.0),
]

harlie = Shape([Primitive(harlie_verts, Color.black)])

x_axis = Shape([Primitive(x_axis_verts, Color.red, GL_LINES)])

y_axis = Shape([Primitive(y_axis_verts, Color.orange, GL_LINES)])
