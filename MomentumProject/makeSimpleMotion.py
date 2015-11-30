import math as mm


def makeSineMotion(initPos, totalFrame, freq, maxDeg):
    for i in range(totalFrame):
        phase = i/float(totalFrame)
        print initPos[0], initPos[1], initPos[2], 0., 0., 0., \
            0., 0., 0.*(-maxDeg*mm.cos(2*mm.pi*phase*freq)/2.-maxDeg/2.), \
            0., 0., 0.*(-maxDeg*mm.cos(2*mm.pi*phase*freq)/2.-maxDeg/2.), \
            0., 0., 0.*(maxDeg*mm.cos(2*mm.pi*phase*freq)/2.+maxDeg/2.), \
            0., 0., 0.*(maxDeg*mm.cos(2*mm.pi*phase*freq)/2.+maxDeg/2.), \
            0., 0., 0.*(maxDeg*mm.cos(2*mm.pi*phase*freq)/2.+maxDeg/2.), \
            0., 0., 0.*(maxDeg*mm.cos(2*mm.pi*phase*freq)/2.+maxDeg/2.)

makeSineMotion([0., 15., 0.], 100, 30, 0.)
