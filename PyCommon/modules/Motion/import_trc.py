import numpy as np
import Math.mmMath as mm
import math


def import_trc(filename):
    trcFile = open(filename, 'r')
    infoNames = []
    infoData = []

    markerName = []
    markerPosi = []

    trcFile.readline()

    def strs2vec3(strs):
        floats = []
        for w in strs:
            if w == '':
                floats.append(0.0)
            elif w == '\r\n':
                pass
            else:
                floats.append(float(w)/200.)
        out = []
        for fidx in range(0, len(floats),3):
            vec = np.array((floats[fidx], floats[fidx+1], floats[fidx+2]))
            R = mm.exp((1,0,0), -math.pi/2)
            vec = np.dot(R, vec)

            out.append(vec)
        return out

    infoNames = trcFile.readline().split()
    infoDatatmp = trcFile.readline().split()

    markerName = trcFile.readline().split()[2:]
    for i in range(0, len(infoNames)):
        try:
            infoData.append(float(infoDatatmp[i]))
        except:
            infoData.append(infoDatatmp[i])

    # X1 Y1 Z1 ...
    trcFile.readline()
    trcFile.readline()

    for i in range(0, int(infoData[2])):
        markerPosi.append(strs2vec3(trcFile.readline().split()[2:]))

    trcFile.close()

    # for i in range(0, len(infoNames)):
    #     print infoNames[i], ": ", infoData[i]
    # print markerName
    # print markerPosi
    return infoNames, infoData, markerName, markerPosi


