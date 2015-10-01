from cvxopt import matrix, solvers
import numpy as np
import numpy.linalg as npl
#import scipy.linalg as npl

# Equality Constrained Least Square
# minimize |C1x-d1|^2 + ... + |Cnx-dn|^2
# subject to Ax=b
class LSE:
    def __init__(self, varNum, consNum):
        self.varNum = varNum
        self.consNum = consNum
    
        self.Cs = []
        self.ds = []
        self.A = None
        self.b = None
        #self.A = []
        #self.b = []
        
    # add |Cix-di|^2
    # C: matrix, d: vector
    def addObjective_matrix(self, Ci, di, w=1.): 
        if Ci.shape[1]!=self.varNum:
            print 'varNum mismatched 1'
            print("Ci.shape[1]", Ci.shape[1])
            print("self.varNum", self.varNum)
            return
        if Ci.shape[0]!=len(di):
            print 'Ci & di mismatched'
            return
        self.Cs.append(w*Ci)
        self.ds.append(w*di)
        
    # subject to Ax=b
    # A: matrix, b: vector
    def setConstraint_matrix(self, A, b):
        '''
        if A.shape[1]!=self.varNum:
            print 'varNum mismatched 2'
            return
        if len(A)!=self.consNum or len(b)!=self.consNum:
            print("len(A)=", len(A))
            print("len(b)=", len(b))
            print("consNum misnatched==", self.consNum)
            return
        '''
        self.A = A
        self.b = b
    
    # subject to Ax=b
    # A: matrix, b: vector
    def addConstraint_matrix(self, A, b):
        #print("b", b)
        #print("b.T", b.T)
        '''
        if A.shape[1]!=self.varNum:
            print 'varNum mismatched 2'
            return
        if len(A)!=self.consNum or len(b)!=self.consNum:
            print 'consNum misnatched'
            return
        '''
        #self.A.append(A)
        #self.b.append(b)
        if self.A == None:
            self.A = A
            self.b = b
        else:
            self.A = np.vstack((self.A, A))
            self.b = np.append(self.b, b)
        
        
    def solve(self):
#        \underset{x}{\operatorname{min}}\left \|  C_{1}x-d_{1}\right \|^{2} + \cdots  + \left \|  C_{n}x-d_{n)}\right \|^{2} \newline
#        subject \; to : Ax-b=0 \newline
#        \newline
#        FONC : \newline
#        2C_{1}^{T}C_{1}x-2C_{1}^{T}d_{1} + \cdots + 2C_{n}^{T}C_{n}x-2C_{n}^{T}d_{n} 
#        
#        + A^
#        
#        {T}\lambda =0 \newline
#        Ax-b=0 \newline
#        \newline
#        \Rightarrow
#        \begin{pmatrix}
#         2C_{1}^{T}C_{1} + \cdots + 2C_{n}^{T}C_{n} & A^{T} \\ 
#         A & 0
#        \end{pmatrix}
#        \begin{pmatrix}
#        x\\ 
#        \lambda
#        \end{pmatrix}
#        =
#        \begin{pmatrix}
#        2C_{1}^{T}d_{1} + \cdots + 2C_{n}^{T}d_{n} \\ 
#        b
#        \end{pmatrix}

        # build system
        # A_large * x_large = b_large
        A11 = sum([2*np.dot(Ci.T, Ci) for Ci in self.Cs])

        A = self.A
        if self.A == None:
            A = np.zeros((1, A11.shape[0]))
        A12 = A.T
        A21 = A
        A22 = np.zeros((A.shape[0], A.shape[0]))
        A_large = np.vstack((np.hstack((A11,A12)), np.hstack((A21,A22))))      

        b1 = sum([2*np.dot(self.Cs[i].T, self.ds[i]) for i in range(len(self.Cs))])
        b2 = self.b
        if self.b == None:
            b2 = (0)
        b_large = np.hstack((b1,b2))        
        
        #x_large = npl.solve(A_large, b_large)
        x_large = npl.lstsq(A_large, b_large)
        if np.isnan(x_large[0][0]):
            print 'nan!!'
            x_large = npl.lstsq(A11, b1)
        #x_large = npl.lstsq(A11, b1)
        
        result = {}
        result['x'] = x_large[0][:self.varNum]
        result['lambda'] = x_large[self.varNum:]
        return result

    def solve2(self):
#        \underset{x}{\operatorname{min}}\left \|  C_{1}x-d_{1}\right \|^{2} + \cdots  + \left \|  C_{n}x-d_{n)}\right \|^{2} \newline
#        subject \; to : Ax-b=0 \newline
#        \newline
#        FONC : \newline
#        2C_{1}^{T}C_{1}x-2C_{1}^{T}d_{1} + \cdots + 2C_{n}^{T}C_{n}x-2C_{n}^{T}d_{n} 
#        
#        + A^
#        
#        {T}\lambda =0 \newline
#        Ax-b=0 \newline
#        \newline
#        \Rightarrow
#        \begin{pmatrix}
#         2C_{1}^{T}C_{1} + \cdots + 2C_{n}^{T}C_{n} & A^{T} \\ 
#         A & 0
#        \end{pmatrix}
#        \begin{pmatrix}
#        x\\ 
#        \lambda
#        \end{pmatrix}
#        =
#        \begin{pmatrix}
#        2C_{1}^{T}d_{1} + \cdots + 2C_{n}^{T}d_{n} \\ 
#        b
#        \end{pmatrix}

        # build system
        # A_large * x_large = b_large
        A11 = sum([2*np.dot(Ci.T, Ci) for Ci in self.Cs])
        A12 = self.A[0].T
        A13 = self.A[1].T
        A21 = self.A[0]
        A31 = self.A[1]

        #A12 = self.A.T
        #A21 = self.A
        
        A22 = np.zeros((len(self.A[0]), len(self.A[0])))
        A23 = np.zeros((len(self.A[0]), len(self.A[0])))
        A32 = np.zeros((len(self.A[0]), len(self.A[0])))
        A33 = np.zeros((len(self.A[0]), len(self.A[0])))      
        #A_large = np.vstack((np.hstack((A11,A12)), np.hstack((A21,A22))))
        A_large = np.vstack( ( np.hstack((A11,A12,A13)), np.hstack((A21,A22,A23)), np.hstack((A31,A32,A33)) ))
                        
        b1 = sum([2*np.dot(self.Cs[i].T, self.ds[i]) for i in range(len(self.Cs))])
        #b2 = self.b
        b2 = self.b[0]
        b3 = self.b[1]
        #b_large = np.hstack((b1,b2))
        b_large = np.hstack((b1,b2, b3))
        x_large = None
        try:
            x_large = npl.solve(A_large, b_large)
        except:
            print "exception!"
            x_large = npl.lstsq(A_large, b_large)
        
        result = {}
        result['x'] = x_large[0][:self.varNum]
        result['lambda'] = x_large[self.varNum:]
        #print(np.allclose(np.dot(A_large, x_large), b_large))
        return result
    
    def clear(self):
        del self.Cs[:]
        del self.ds[:]
        self.A = None
        self.b = None
        #del self.A[:]
        #del self.b[:]

# Equality and Inequality Constrained Least Square
# minimize |C1x-d1|^2 + ... + |Cnx-dn|^2
# subject to Gx <= h
#            Ax  = b

class QP:
    def __init__(self):
        self.Cs = []
        self.ds = []
        self.G = None
        self.h = None
        self.A = None
        self.b = None

    # add |Cix-di|^2
    # C: matrix, d: vector
    # all Ci has to be agree both rows and cols
    def addObjective(self, Ci, di, w=1.): 
        self.Cs.append(w*Ci)
        self.ds.append(w*di)
        
    # subject to Ax=b
    # A: matrix, b: vector
    def addEqualityConstraint(self, A, b):
        if self.A == None:
            self.A = A
            self.b = b
        else:
            self.A = np.vstack((self.A, A))
            self.b = np.append(self.b, b)

    # subject to Gx<=h
    # G: matrix, h: vector
    def addInequalityConstraint(self, G, h):
        if self.G == None:
            self.G = G
            self.h = h
        else:
            self.G = np.vstack((self.G, G))
            self.h = np.append(self.h, h)

    def solve(self):
        # build system
        Q_np = sum([np.dot(Ci.T, Ci) for Ci in self.Cs])
        p_np = sum([(-1.)*np.dot(self.Cs[i].T, self.ds[i]) for i in range(len(self.Cs))])
        
        Q = matrix(Q_np)
        p = matrix(p_np)
        G = matrix(self.G)
        h = matrix(self.h)
        A = matrix(self.A)
        b = matrix(self.b)
        
        solvers.options['show_progress'] = False
        solvers.options['maxiter'] = 1000
        x_large = solvers.qp(Q, p, G, h, A, b)['x']
        return x_large
            
        
        #result = {}
        #result['x'] = x_large[:self.varNum]
        #if self.A!=None:
            #result['lambda'] = x_large[self.varNum:]
        #return result
    
    def clear(self):
        del self.Cs[:]
        del self.ds[:]
        self.G = None
        self.h = None
        self.A = None
        self.b = None

    
    
if __name__ == '__main__':
    import psyco; psyco.full()
    
    def test_LSE():
        
        # minimize :    f(x,y) = x^2 + y^2 + z^2
        # subject to :    x + y + z = 1
        
        # convert to form of
        # min |Cx-d|^2     s.t Ax=b
        
        # minimize |[1 0 0]*[x] - [0]|^2 
        #           [0 1 0] [y]   [0]
        #           [0 0 1] [z]   [0]
        # subject to  [1 1 1] * [x y z].T = 1
        
        p = LSE(3, 1)
        p.addObjective_matrix(np.eye(3), np.zeros(3))
        p.setConstraint_matrix(np.array([[1,1,1]]), [1])
        r = p.solve()
        print r 
        
        #    subject to :    x + y = 1
        #                    y + z = 1
        #                    x + z = 1
        p = LSE(3, 3)
        p.addObjective_matrix(np.eye(3), np.zeros(3))
        p.setConstraint_matrix(np.array([[1,1,0],
                                         [0,1,1],
                                         [1,0,1]]), [1,1,1])
        r = p.solve()
        print r 
        
        #    subject to :    x + y = 1
        #                    y + z = 1
        p = LSE(3, 2)
        p.addObjective_matrix(np.eye(3), np.zeros(3))
        p.setConstraint_matrix(np.array([[1,1,0],
                                         [0,1,1]]), [1,1])
        r = p.solve()
        print r 
    
        
    test_LSE()
