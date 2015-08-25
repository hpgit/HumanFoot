from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GLE import *
import ode, numpy

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Motion.ysMotion as ym

# RendererContext
NORMAL_FLAT = 0
NORMAL_SMOOTH = 1

# RendererContext, OdeModelRenderer
POLYGON_LINE = 0
POLYGON_FILL = 1

# JointMotionRenderer
LINK_LINE = 0
LINK_BONE = 1
LINK_SOLIDBOX = 2
LINK_WIREBOX = 3

# RenderContext. PointsRenderer
POINT_POINT = 0
POINT_CROSS = 1
POINT_CUBE = 2 

#SELECTION_COLOR = (10,10,.7)
SELECTION_COLOR = (2550,2550,178)

RENDER_OBJECT = 0
RENDER_SHADOW = 1
RENDER_REFLECTION = 2

class Renderer:
    def __init__(self, target, color):
        self.rc = RenderContext()
        self.totalColor = color
        self.selectedElement = None
        self.shadowColor = (150,150,150)
    def render(self, renderType):
        print "Renderer.render() : Must subclass me"
        raise NotImplementedError
    
class JointMotionRenderer(Renderer):
    def __init__(self, target, color = (0,255,255), linkStyle = LINK_LINE, lineWidth=1.):
        Renderer.__init__(self, target, color)
        self.motion = target
        self.renderFrames = None
        self.setLinkStyle(linkStyle)
        self.rc.setLineWidth(lineWidth)
    def setLinkStyle(self, linkStyle):
        self.linkStyle = linkStyle
        if self.linkStyle == LINK_WIREBOX:
            self.rc.setPolygonStyle(POLYGON_LINE)
        else:
            self.rc.setPolygonStyle(POLYGON_FILL)
    def render(self, renderType=RENDER_OBJECT):
        if len(self.motion) > 0:
            self.rc.beginDraw()
            if renderType==RENDER_SHADOW:
                glColor3ubv(self.shadowColor)
            else:
                glColor3ubv(self.totalColor)
            if self.renderFrames==None:
                posture = self.motion[self.motion.frame]
                self.renderJointPosture(posture)
            else:
                for renderFrame in self.renderFrames:
                    posture = self.motion[renderFrame]
                    self.renderJointPosture(posture)
    def renderJointPosture(self, posture):
        joint = posture.skeleton.root
        glPushMatrix()
        glTranslatef(posture.rootPos[0], posture.rootPos[1], posture.rootPos[2])
        self._renderJoint(joint, posture)
        glPopMatrix()
    def _renderJoint(self, joint, posture):
        glPushMatrix()
        glTranslatef(joint.offset[0],joint.offset[1],joint.offset[2])
#        glMultMatrixf(mm.R2T(posture.localRMap[joint.name]).transpose())
        glMultMatrixf(mm.R2T(posture.localRs[posture.skeleton.getElementIndex(joint.name)]).transpose())
                
#        if joint.name in self.partColors:
#            color = self.partColors[joint.name]
#        else:
#            color = self.totalColor
            
        if joint == self.selectedElement:
            glColor3ubv(SELECTION_COLOR)
            ygh.beginDraw()
            ygh.drawCoordinate()
            ygh.endDraw()
            
        # 1
#        ygh.drawPoint((0,0,0), color)

        if self.linkStyle == LINK_LINE:
            self.rc.drawPoint((0,0,0))
            for childJoint in joint.children:
                self.rc.drawLine((0,0,0), childJoint.offset)
                
        elif self.linkStyle == LINK_BONE:
#            self.rc.drawPoint((0,0,0))
            self.rc.drawLine((-.05,0,0), (.05,0,0))
            for childJoint in joint.children:
                self.rc.drawLine((0,0,0), childJoint.offset)
        
        elif self.linkStyle == LINK_SOLIDBOX or self.linkStyle == LINK_WIREBOX:        
            if len(joint.children) > 0:
                glPushMatrix()
                
                offset = numpy.array([0.,0.,0.])
                for childJoint in joint.children:
                    offset += childJoint.offset
                offset = offset/len(joint.children)
                
                defaultBoneV = numpy.array([0,0,mm.length(offset)])
                boneT = mm.R2T(mm.getSO3FromVectors(defaultBoneV, offset))
                glMultMatrixf(boneT.transpose())
        
                glTranslatef(-.05,-.05,0)
#                ygh.beginDraw()
#                ygh.drawCoordinate()
#                ygh.endDraw()
        
                self.rc.drawBox(.1,.1,mm.length(offset))
                glPopMatrix()
        
        if joint == self.selectedElement:
            glColor3ubv(self.totalColor)
        
        for childJoint in joint.children:
            self._renderJoint(childJoint, posture)
        glPopMatrix()
        
#===============================================================================
# # debugging renderers
#===============================================================================
class PointsRenderer(Renderer):
    def __init__(self, points, color = (255,0,0), pointStyle = POINT_CROSS):
        Renderer.__init__(self, points, color)
        self.points = points
        self.pointStyle = pointStyle
        self.rc.setLineWidth(2.)
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        glColor3ubv(self.totalColor)
        for point in self.points:
            if point!=None:
                if self.pointStyle==POINT_POINT:
                    self.rc.drawPoint(point)
                elif self.pointStyle==POINT_CROSS:
                    self.rc.drawCross(point)
                elif self.pointStyle==POINT_CUBE:
                    self.rc.drawCube(point)

class VectorsRenderer(Renderer):
    def __init__(self, vectors, origins, color = (255,0,0), lineWidth=.02):
        Renderer.__init__(self, vectors, color)
        self.vectors = vectors
        self.origins = origins
        self.lineWidth = lineWidth
        self.rc.setLineWidth(lineWidth)
    def render(self, renderType=RENDER_OBJECT):
        glColor3ubv(self.totalColor)
        self.rc.beginDraw()
        for i in range(len(self.vectors)):
            if self.vectors[i] != None and self.origins[i] != None:
                origin = self.origins[i]; vector = self.vectors[i]
                self.rc.drawLine(origin, (origin[0]+vector[0],origin[1]+vector[1],origin[2]+vector[2]))
            
class ForcesRenderer(Renderer):
    def __init__(self, forces, points, color=(255,0,0), ratio=1., lineWidth=.02, fromPoint=True):
        Renderer.__init__(self, None, color)
        self.forces = forces
        self.points = points
        self.ratio = ratio
        self.lineWidth = lineWidth
        self.fromPoint = fromPoint
        self.rc.setNormalStyle(NORMAL_SMOOTH)
    def render(self, renderType=RENDER_OBJECT):
        self.rc.beginDraw()
        glColor3ubv(self.totalColor)
        for i in range(len(self.forces)):
            if self.forces[i]!=None and self.points[i]!=None:
                if self.fromPoint==False:
                    self.rc.drawArrow(None, self.points[i], mm.v3_scale(self.forces[i], self.ratio), self.lineWidth)
                else:
                    self.rc.drawArrow(self.points[i], None, mm.v3_scale(self.forces[i], self.ratio), self.lineWidth)

#===============================================================================
# # common class
#===============================================================================
class RenderContext:
    def __init__(self):
        self.quad = gluNewQuadric()
        gleSetNumSides(12)
        
        self.setPolygonStyle(POLYGON_FILL)
        self.setNormalStyle(NORMAL_FLAT)
        self.setLineWidth(1.)
        self.crossLength = .1
        
    def __del__(self):
        gluDeleteQuadric(self.quad)
        
    def setPolygonStyle(self, polygonStyle):
        self.polygonStyle = polygonStyle
        if polygonStyle == POLYGON_LINE:
            gluQuadricDrawStyle(self.quad, GLU_LINE)
        elif polygonStyle == POLYGON_FILL:
            gluQuadricDrawStyle(self.quad, GLU_FILL)
            
    def setNormalStyle(self, normalStyle):
        self.normalStyle = normalStyle
        if normalStyle == NORMAL_FLAT:
            gluQuadricDrawStyle(self.quad, GLU_FLAT)
        elif normalStyle == NORMAL_SMOOTH:
            gluQuadricDrawStyle(self.quad, GLU_SMOOTH)
            
    def setLineWidth(self, lineWidth):
        self.lineWidth = lineWidth
            
    def beginDraw(self):
        if self.polygonStyle == POLYGON_LINE:
            glPolygonMode(GL_FRONT, GL_LINE)
        elif self.polygonStyle == POLYGON_FILL:
            glPolygonMode(GL_FRONT, GL_FILL)
        
        if self.normalStyle == NORMAL_FLAT:
            gleSetJoinStyle(TUBE_NORM_FACET | TUBE_JN_CAP | TUBE_JN_CUT)
        elif self.normalStyle == NORMAL_SMOOTH:
            gleSetJoinStyle(TUBE_NORM_EDGE | TUBE_JN_CAP | TUBE_JN_CUT)
            
        glLineWidth(self.lineWidth)
            
    #===============================================================================
    # draw primitives at origin    
    #===============================================================================
    def drawBox(self, lx, ly, lz):
        glPushMatrix()
        glTranslated(lx/2.,ly/2.,lz/2.)
        glScale(lx, ly, lz)
        if self.polygonStyle == POLYGON_LINE:
            glutWireCube(1)
        else:
            glutSolidCube(1)
        glPopMatrix()
    def drawCylinder(self, radius, length_z):
        gluCylinder(self.quad, radius, radius, length_z, 16, 1)

    #===============================================================================
    # draw primitives at its position        
    #===============================================================================
    def drawPoint(self, point):
        glPointSize(3.0)
        glBegin(GL_POINTS)
        glVertex3fv(point)
        glEnd()
        
    def drawCross(self, point):
        glPushMatrix()
        glTranslatef(point[0], point[1], point[2])
        glBegin(GL_LINES)
        crossLength = self.crossLength
        glVertex3f(crossLength/2.,0,0)    # x
        glVertex3f(-crossLength/2.,0,0)
        glVertex3f(0,crossLength/2.,0)    # y
        glVertex3f(0,-crossLength/2.,0)
        glVertex3f(0,0,crossLength/2.)    # z
        glVertex3f(0,0,-crossLength/2.)
        glEnd()
        glPopMatrix()
        
    def drawCube(self, point):
        ygh.beginDraw()
        ygh.drawPoint(point)
        ygh.endDraw()
        glPushMatrix()
        glTranslated(point[0], point[1], point[2])
        glutWireCube(.1)
        glPopMatrix()
            
    def drawLine(self, startPos, endPos):
        glBegin(GL_LINES)
#        glVertex3fv(startPos)
#        glVertex3fv(endPos)
        glVertex3f(startPos[0], startPos[1], startPos[2])
        glVertex3f(endPos[0], endPos[1], endPos[2])
        glEnd()
        
    def draw2DArrow(self, startPos, endPos, vector=None, lineWidth=.02):
        if vector==None:
            vector = [endPos[i]-startPos[i] for i in range(3)]
        elif startPos==None:
            startPos = [endPos[i]-vector[i] for i in range(3)]
        
#        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        
        glDisable(GL_CULL_FACE)
        glPushMatrix()
        
        length = mm.length(vector)
        arrowT = mm.Rp2T(mm.getSO3FromVectors((length,0,0), vector), startPos)
        glMultMatrixf(arrowT.transpose())
        
        triWidth = lineWidth * 3
        triLength = triWidth * 1.2
        
        angles = [0, 90]
        for angle in angles:
            glRotatef(angle, 1,0,0)
        
            # line part
            glBegin(GL_QUADS)        
            glVertex3f(0,0,lineWidth/2)
            glVertex3f(0,0,-lineWidth/2)
            glVertex3f(length - triLength,0,-lineWidth/2)
            glVertex3f(length - triLength,0,+lineWidth/2)
            glEnd()
            
            # triangle part
            glBegin(GL_TRIANGLES)
            glVertex3f(length - triLength, 0, triWidth/2)
            glVertex3f(length - triLength, 0, -triWidth/2)
            glVertex3f(length, 0, 0)
            glEnd()
    
        glPopMatrix()
        glEnable(GL_CULL_FACE)
        
    def drawArrow(self, startPos, endPos, vector=None, lineWidth=.02):
        if vector==None:
            vector = [endPos[i]-startPos[i] for i in range(3)]
        elif startPos==None:
            startPos = [endPos[i]-vector[i] for i in range(3)]
        
        length = mm.length(vector)
        if length==0.: return

        glPushMatrix()
        
        arrowT = mm.Rp2T(mm.getSO3FromVectors((length,0,0), vector), startPos)
        glMultMatrixf(arrowT.transpose())
        
        triWidth = lineWidth * 3
        triLength = triWidth * 1.2
        
        # line + cone all parts
        glePolyCone(((0,0,0), (0,0,0), (length-triLength,0,0), (length-triLength,0,0), (length,0,0), (length,0,0)), None, 
                    (lineWidth/2., lineWidth/2., lineWidth/2., triWidth/2., 0, 0))
        
        glPopMatrix()
    
    def drawCircularArrow(self, startPos, endPos, rotVec=None, lineWidth=.02, radius=.1):
        if rotVec==None:
            rotVec = [endPos[i]-startPos[i] for i in range(3)]
        elif startPos==None:
            startPos = [endPos[i]-rotVec[i] for i in range(3)]
        
        length = mm.length(rotVec)
        if length==0.: return

        glPushMatrix()
        
        axisT = mm.Rp2T(mm.getSO3FromVectors((0,0,length), rotVec), startPos)
        glMultMatrixf(axisT.transpose())
        
        triWidth = lineWidth * 3
        triLength = triWidth * 1.2
        
        # axis
#        self.drawLine((0,0,0), (0,0,length))
        glePolyCylinder(((0,0,0), (0,0,0), (0,0,length), (0,0,length)), None, lineWidth/4.)
        
        # circular line part
#        gleHelicoid( rToroid , startRadius , drdTheta , startZ , dzdTheta , 
#                     startXform , dXformdTheta , startTheta , sweepTheta )
        sweepTheta = 2*math.pi*length*mm.DEG
        gleHelicoid( lineWidth/2., radius,       0.,        0.,    radius,
                        None,         None,            0.,     sweepTheta)
        
        # cone part
        glPushMatrix()
        glRotatef(sweepTheta, 0,0,1)
        glTranslatef(radius, 0, radius * (sweepTheta/360.))
        glRotatef(-90, 1,0,0)
        glePolyCone(((0,0,0), (0,0,0), (0,0,triLength), (0,0,triLength)), None, 
                    (triWidth/2., triWidth/2., 0, 0))
        glPopMatrix()
        
        glPopMatrix()
    
    def renderSelectedOdeGeom(self, geom, color):
        if type(geom) == ode.GeomBox:
            lx, ly, lz = geom.getLengths()
            x,y,z = geom.getPosition()
            R = geom.getRotation()
            se3 = [R[0], R[3], R[6], 0.,
                   R[1], R[4], R[7], 0.,
                   R[2], R[5], R[8], 0.,
                   x, y, z, 1.0]
            glPushMatrix()
            glMultMatrixd(se3)
            ygh.drawCoordinate(color)
            glScaled(1.1,1.1,1.1)
            glTranslated(-lx/2.,-ly/2.,-lz/2.)
            self.drawBox(lx, ly, lz)
            glPopMatrix()
            
    def renderOdeGeom(self, geom):
        if type(geom) == ode.GeomBox:
            lx, ly, lz = geom.getLengths()
            x,y,z = geom.getPosition()
            R = geom.getRotation()
            se3 = [R[0], R[3], R[6], 0.,
                   R[1], R[4], R[7], 0.,
                   R[2], R[5], R[8], 0.,
                   x, y, z, 1.0]
            glPushMatrix()
            glMultMatrixd(se3)
            glTranslated(-lx/2.,-ly/2.,-lz/2.)
            self.drawBox(lx, ly, lz)
            glPopMatrix()
            
        elif type(geom) == ode.GeomCapsule:
            radius, length_z = geom.getParams()
            
            x,y,z = geom.getPosition()
            R = geom.getRotation()
            se3 = [R[0], R[3], R[6], 0.,
                   R[1], R[4], R[7], 0.,
                   R[2], R[5], R[8], 0.,
                   x, y, z, 1.0]
            glPushMatrix()
            glMultMatrixd(se3)
            glTranslated(0,0,-length_z/2.)
            self.drawCylinder(radius, length_z)
            glPopMatrix()
    
        elif type(geom) == ode.GeomSphere:
            radius = geom.getRadius()
        
        elif type(geom) == ode.GeomPlane:
            (a, b, c), d = geom.getParams()
            glPushMatrix()
            glTranslatef(0,d,0)
            glScale(10,0,10)
            glutWireCube(1)
            glPopMatrix()
            
        elif type(geom) == ode.GeomRay:
            length = geom.getLength()
            

