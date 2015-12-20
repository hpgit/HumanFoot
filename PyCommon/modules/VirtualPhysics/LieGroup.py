import numpy as np
import numpy.linalg as npl
import math as m


class DimNotSupportedError(Exception):
    pass


class Vec3(np.ndarray):

    def __new__(cls, *args):
        if len(args) == 0:
            obj = np.asarray(np.zeros(3)).view(cls)
            return obj
        elif len(args) == 1:
            if isinstance(args[0], float):
                obj = np.asarray(args[0] * np.ones(3)).view(cls)
                return obj
            elif len(args[0]) == 3:
                obj = np.asarray(np.array(args[0])).view(cls)
                return obj

            raise DimNotSupportedError

        elif len(args) == 3:
            if isinstance(args[0], float) and isinstance(args[1], float) and isinstance(args[2], float):
                obj = np.asarray(np.array([args[0], args[1], args[2]])).view(cls)
                return obj
            pass

        raise DimNotSupportedError

    def Normalize(self):
        l = npl.norm(self)
        if l != 0.:
            self /= l
        return l

    def Unit(self):
        l = npl.norm(self)
        if l != 0.:
            return self/l
        else:
            raise ZeroDivisionError

class Axis:
    pass

class se3:
    pass

class dse3:
    pass

class Quat:
    pass

#TODO:
class SO3:
    def __new__(cls, *args):
        if len(args) == 0:
            obj = np.asarray(np.eye(3, 3)).view(cls)
            return obj
        elif len(args) == 1:
            if (args[0].shape is not None) and (len(args[0].shape) == 2):
                if (args[0].shape[0] == 3) and (args[0].shape[1] == 3) :
                    obj = np.asarray(np.array(args)).view(cls)
                    return obj

            raise DimNotSupportedError

        elif len(args) == 9:
            if all(isinstance(arg, float) for arg in args):
                R = np.array(args).reshape((3,3))
                obj = np.asarray(R).view(cls)
                return obj

        raise DimNotSupportedError

    def FromAngleAxis(self, v):
        pass

    def FromQuat(self, q):
        pass

    def FromEuler(self, e):
        pass

    def RotX(self, angle):
        return SO3(np.array([[1., 0., 0.], [0., m.cos(angle), -m.sin(angle)], [0., m.sin(angle), m.cos(angle)]]))

    def RotY(self, angle):
        return SO3(np.array([[1., 0., 0.], [0., m.cos(angle), -m.sin(angle)], [0., m.sin(angle), m.cos(angle)]]))

    def RotZ(self, angle):
        return SO3(np.array([[1., 0., 0.], [0., m.cos(angle), -m.sin(angle)], [0., m.sin(angle), m.cos(angle)]]))

class SE3(np.ndarray):
    def __new__(cls, *args):
        if len(args) == 0:
            obj = np.asarray(np.eye(4, 4)).view(cls)
            return obj
        elif len(args) == 1:
            if len(args[0]) == 3:
                T = np.eye(4,4)
                T[0, 3] = args[0][0]
                T[1, 3] = args[0][1]
                T[2, 3] = args[0][2]
                obj = np.asarray(T).view(cls)
                return obj
            elif (args[0].shape is not None) and (len(args[0].shape) == 2):
                if (args[0].shape[0] == 4) and (args[0].shape[1] == 4) :
                    obj = np.asarray(args[0]).view(cls)
                    return obj
                elif (args[0].shape[0] == 3) and (args[0].shape[1] == 3) :
                    T = np.eye(4)
                    T[0:3, 0:3] = np.array(args)
                    obj = np.asarray(args[0]).view(cls)
                    return obj

            raise DimNotSupportedError

        elif len(args) == 9:
            if all(isinstance(arg, float) for arg in args):
                T = np.eye(4)
                T[0:3, 0:3] = np.array(args).reshape((3,3))
                obj = np.asarray(T).view(cls)
                return obj

        elif len(args) == 12:
            if all(isinstance(arg, float) for arg in args):
                T = np.eye(4)
                T[0:3, 0:4] = np.array(args).reshape((3,4))
                obj = np.asarray(T).view(cls)
                return obj

        raise DimNotSupportedError

    def __mul__(self, other):
        if isinstance(other, Vec3):
            return Vec3(np.dot(self[0:3, 0:3], other)) + Vec3(self[0:3, 3])
        else:
            super(SE3, self).__mul__(other)

    def GetVec3(self):
        return Vec3(self[0:3, 3])

    def GetSO3(self):
        return SO3(self[0:3, 0:3])

class Inertia:
#     Inertia BoxInertia(scalar density, const Vec3 &size);
# Inertia SphereInertia(scalar density, scalar rad);
# Inertia CylinderInertia(scalar density, scalar rad, scalar height);
# Inertia TorusInertia(scalar density, scalar ring_rad, scalar tube_rad);
#
# 	/*!
# 		constructor
# 	*/
# 	explicit		 Inertia(scalar mass, scalar Ixx, scalar Iyy, scalar Izz);
#
# 	explicit		 Inertia(scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar, scalar);
    def __new__(cls, *args):
        if len(args) == 0:
            obj = np.asarray(np.zeros(10)).view(cls)
            return obj
        elif len(args) == 1:
            if isinstance(args[0], float):
                I = np.zeros(10)
                I[9] = I[0] = I[1] = I[2] = args[0]
                obj = np.asarray(I).view(cls)
                return obj
            elif len(args[0]) == 4:
                I = np.zeros(10)
                I[9] = args[0][0]
                I[0:3] = args[0][1:4]
                obj = np.asarray(I).view(cls)
                return obj
            elif len(args[0]) == 10:
                obj = np.asarray(np.array(args[0])).view(cls)
                return obj

            raise DimNotSupportedError

        elif len(args) == 4:
            if all(isinstance(arg, float) for arg in args) :
                I = np.zeros(10)
                I[9] = args[0]
                I[0] = args[1]
                I[1] = args[2]
                I[2] = args[3]
                obj = np.asarray(I).view(cls)
                return obj
        elif len(args) == 10:
            if all(isinstance(arg, float) for arg in args) :
                obj = np.asarray(np.array(args)).view(cls)
                return obj

        raise DimNotSupportedError

    def GetMass(self):
        return self[9]


def main():
    v = Vec3(1., 2., 1.)
    v1 = np.array([1., 3., 1.])
    v3 =  v1+Vec3(1., 2., 1.)


if __name__ == '__main__':
    main()