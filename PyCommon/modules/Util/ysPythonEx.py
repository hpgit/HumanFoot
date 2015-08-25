from datetime import datetime
import operator as op

def frange(start, end=None, inc=None):
    "A range function, that does accept float increments..."

    if end == None:
        end = start + 0.0
        start = 0.0

    if inc == None:
        inc = 1.0

    L = []
    while 1:
        next = start + len(L) * inc
        if inc > 0 and next >= end:
            break
        elif inc < 0 and next <= end:
            break
        L.append(next)
        
    return L

def mrange(minvec, maxvec=None):  
    if maxvec is None:
        maxvec = minvec
        minvec = [0] * len(maxvec)
    vec = list(minvec)
    unitpos = len(vec) - 1
    maxunit = maxvec[unitpos]
    _tuple = tuple
    while 1:
        if vec[unitpos] == maxunit:
            i = unitpos
            while vec[i] == maxvec[i]:
                vec[i] = minvec[i]
                i -= 1
                if i == -1:
                    return            
                vec[i] += 1
        yield _tuple(vec)
        vec[unitpos] += 1
         
def map_dict(func, dic1, dic2):
    dic = {}
    for key in dic1:
        dic[key] = func(dic1[key], dic2[key])
    return dic

def print_seq(seq, singleLine=False):
    if singleLine:
        print '[',
        for i in range(len(seq)):
            print seq[i],
        print ']'
    else:
        for i in range(len(seq)):
            print '[%d]'%i, seq[i]
            
def print_dict(dic, singleLine=False):
    if singleLine:
        print '{',
        for key, value in dic.items():
            print '%s'%repr(key), ':', value, ',' ,
        print '}'
    else:
        for key, value in dic.items():
            print '[%s]'%repr(key), value
            
def updateObject(object, otherObject):
    object.__dict__ = otherObject.__dict__
    
def getLogTimeString():
    return datetime.today().strftime('%y%m%d_%H%M%S')

def getReverseDict(dic):
    rdic = {}
    for key, value in dic.items():
        rdic[value] = key
    return rdic

# input : 
# inList == [10,20,30]
# outList == [None,None,None,None,None,None]
# repeatNums = [3,2,1]
# output : 
# outLIst == [10,10,10,20,20,30]
def repeatListElements(inList, outList, repeatNums):
    index = 0
    for i in range(len(inList)):
        for j in range(repeatNums[i]):
            outList[index] = inList[i]
            index += 1
    
# input:
# inList == [1,2,3,4,5,6]
# outList == [None, None]
# sumNums == [3,3]       
# output:
# outList == [6, 15] 
def sumListElements(inList, outList, sumNums, addOperation=op.iadd, additiveIdentity=0):
    index = 0
    for i in range(len(sumNums)):
        sum = additiveIdentity
        for j in range(sumNums[i]):
            sum = addOperation(sum, inList[index])
            index += 1
        outList[i] = sum

def makeFlatList(totalDOF):
    return [None]*totalDOF
def makeNestedList(dofs):
    ls = [None]*len(dofs)
    for i in range(len(dofs)):
        ls[i] = [None]*dofs[i]
    return ls
def flatten(nestedList, flatList):
    i = 0
    for a in range(len(nestedList)):
        for b in range(len(nestedList[a])):
            flatList[i] = nestedList[a][b]
            i += 1
def nested(flatList, nestedList):
    i = 0
    for a in range(len(nestedList)):
        for b in range(len(nestedList[a])):
            nestedList[a][b] = flatList[i]
            i += 1
