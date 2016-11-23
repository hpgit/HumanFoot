import sys, numpy, ode, math, copy

class WorldConfig:
    """
    :type timeStep: float
    :type gravity: tuple[float]
    :type planeHeight: float
    :type useDefaultContactModel:  bool
    :type lockingVel: float
    """
    def __init__(self):
        self.timeStep = 0.001
        
        self.gravity = (0, -9.8, 0)
        # self.gravity = (0, 0, 0)
        self.planeHeight = 0.0
        self.useDefaultContactModel = True
        self.lockingVel = 0.02
        #self.lockingVel = 0.05
        
        
        ##########################################33
        # for ODE
        # below values are ODE default values
        self.ContactSurfaceLayer = 0.0
        self.ContactMaxCorrectingVel = ode.Infinity
        self.ERP = 0.2
        self.CFM = 1E-5
        ##########################################33


class Material:
    def getMass(self):
        print("Please Specify Material Type")
        return -1

class BoxMaterial(Material):
    def __init__(self, density=1000., length=1., width=1., height=1.):
        self.density = density
        self.length = length
        self.width = width
        self.height = height

    def getMass(self):
        return self.density * self.length * self.width * self.height

class CapsuleMaterial(Material):
    def __init__(self, density=1000., radius=1., height=1.):
        self.density = density
        self.radius = radius
        self.height = height

    def getMass(self):
        return self.density * self.radius * self.radius * math.pi * self.height


class Node:
    """
    :type name : str
    :type mass : float
    :type offset : tuple[float]
    :type length : float
    :type width : float
    :type geom : str
    :type jointType : str
    :type jointAxes : list[list[float]]
    :type geoms : list[str]
    :type geomTs : list[np.array]
    :type geomMaterial : list[CapsuleMaterial | BoxMaterial]
    """
    def __init__(self, name):
        self.name = name
        self.mass = None
        self.offset = (0,0,0)
        self.length = None
        self.width = None
        self.geom = 'MyBox'
        self.jointType = 'B'

        self.jointAxes = []
        self.jointLoStop = -ode.Infinity
        self.jointHiStop = ode.Infinity

        self.density = 1000 # 1000 kg/m^3 = 1 g/cm^3 : density of liquid water at 4'C
        self.boneRatio = 1.
        self.Kp = 100
        self.Kd = 5

        ##########################################33
        # for ODE
        # below values are ODE default values
        self.contactMode = ode.ContactBounce
        self.contactMu = ode.Infinity
        self.contactBounce = 0.1
        self.contactSoftERP = 0.0
        self.contactSoftCFM = 0.0
        ##########################################33

        self.geoms = []
        self.geomTs = []
        self.geomMaterial = []

    def addGeom(self, geom, geomT, material):
        self.geoms.append(geom)
        self.geomTs.append(geomT)
        self.geomMaterial.append(material)


class ModelConfig:
    """
    :type nodes: dict(str, Node)
    """
    def __init__(self):
        self.nodes = {}
        # tempNode = ModelConfig.Node('')
        tempNode = Node('')
        self.defaultDensity = tempNode.density
        self.defaultBoneRatio = tempNode.boneRatio
        self.defaultKp = tempNode.Kp
        self.defaultKd = tempNode.Kd
        self.defaultJointAxes = tempNode.jointAxes
        self.defaultJointLoStop = tempNode.jointLoStop
        self.defaultJointHiStop = tempNode.jointHiStop
        self.defaultContactMode = tempNode.contactMode
        self.defaultContactMu = tempNode.contactMu
        self.defaultContactBounce = tempNode.contactBounce
        self.defaultContactSoftERP = tempNode.contactSoftERP
        self.defaultContactSoftCFM = tempNode.contactSoftCFM
    def addNode(self, name):
        # node = ModelConfig.Node(name)
        node = Node(name)
        node.density = self.defaultDensity
        node.boneRatio = self.defaultBoneRatio
        node.Kp = self.defaultKp
        node.Kd = self.defaultKd
        node.jointAxes = self.defaultJointAxes
        node.jointLoStop = self.defaultJointLoStop
        node.jointHiStop = self.defaultJointHiStop
        node.contactMode = self.defaultContactMode
        node.contactMu = self.defaultContactMu
        node.contactBounce = self.defaultContactBounce
        node.contactSoftERP = self.defaultContactSoftERP
        node.contactSoftCFM = self.defaultContactSoftCFM
        self.nodes[name] = node
        return node
    def getNode(self, name):
        """

        :type name: str
        :rtype: Node
        """
        return self.nodes[name]
    def hasNode(self, name):
        return name in self.nodes
    def setNodeCopy(self, name, node):
        nodeCopy = copy.deepcopy(node)
        nodeCopy.name = name 
        self.nodes[name] = nodeCopy
    def delNode(self, name):
        del self.nodes[name]
    def adjustTotalMass(self, totalMass):
        oldTotalMass = 0.
        for node in self.nodes.values():
            oldTotalMass += node.mass
        for node in self.nodes.values():
            node.mass *= (totalMass / oldTotalMass)
