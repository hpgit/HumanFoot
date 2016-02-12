import math
import numpy as np
import LemkeSolverBase


class LemkeLCPSolver(LemkeSolverBase):
    class BasisState:
        def __init__(self):
            self.zactive = 0
            self.driveVar = None

        def equals(self, state):
            return (self.zactive == state.zactive) and (self.driveVar == state.driveVar)

        def set(self, state):
            self.zactive = state.zactive
            self.driveVar = state.driveVar

        def init(self, drive):
            self.zactive = 0
            self.driveVar = drive

        def pivot(self, dropping, entering):
            if entering.isZ():
                self.zactive |= (1L << entering.idx)
            if dropping.isZ() and not dropping.isZ0():
                self.zactive &= ~(1L << dropping.idx)
            self.driveVar = dropping.complement

            # def toString(self):
            #     StringBuffer sbuf = new StringBuffer(256)
            #     NumberFormat fmt = new NumberFormat("%x")
            #     for i in range(0, 64):
            #         if (zactive & (1L<<i)) != 0:
            #             sbuf.append (" z")
            #             sbuf.append ((i+1))
            #     if driveVar != null:
            #         sbuf.append (" z0 (" + driveVar.getName() + ")")
            #     return sbuf.toString()
    '''
    //  private Variable[] variableList = new Variable[0];
    private Variable[] zVars = new Variable[0];
    private Variable[] wVars = new Variable[0];
    private Variable[] basicVars;
    private Variable[] nonBasicVars;
    private Variable z0Var;
    private Variable wzVar;
    private int nonCompIdx = -1;

    private double[] Mv;



    // stuff for incremental pivoting

    private double[] Mbuf;
    private double[] qbuf;
    private int msize;
    private MatrixNd Raa = new MatrixNd(0,0);
    //  private double[] RaaBuf = new double[0];
    private VectorNd avec0 = new VectorNd(0);
    private VectorNd avec1 = new VectorNd(0);
    private VectorNd avec2 = new VectorNd(0);
    private VectorNd avec3 = new VectorNd(0);
    private Variable[] zAlpha = new Variable[0];
    private Variable[] wAlpha = new Variable[0];
    private int asize = 0;

    private double[] qv = new double[0];
    private double[] mv = new double[0];
    private double[] iv = new double[0];
    private double[] cvec = new double[0];

    private double[] qvNew = new double[0];
    //  private Interval[] ratioIntervals = new Interval[0];
    private int[] columnList = new int[0];

    private boolean fullSolve = false;
    private boolean incrementalPivoting = false;
    private MatrixNd Morig = new MatrixNd(0,0);
    private VectorNd qorig = new VectorNd(0);
    private MatrixNd Basis = new MatrixNd(0,0);
    private MatrixNd InvBasis = new MatrixNd(0,0);
    private VectorNd col = new VectorNd(0);

    private BasisState[] basisHistory = new BasisState[0];
    private BasisState currentBasis = new BasisState();
    private BasisState proposedBasis = new BasisState();

    private String[] zvarNames = null;
    private String[] wvarNames = null;
    '''
    def __init__(self):
        #  self.variableList = new Variable[0]
        self.zVars = []
        self.wVars = []
        self.basicVars = self.Variable()
        self.nonBasicVars = self.Variable()
        self.z0Var = self.Variable()
        self.wzVar = self.Variable()
        self.nonCompIdx = -1

        self.Mv = None

        # stuff for incremental pivoting
        self.Mbuf = None
        self.qbuf = None
        self.msize = 0
        self.Raa = np.zeros((0, 0))
        #  self.RaaBuf = new double[0]
        self.avec0 = np.zeros((0,))
        self.avec1 = np.zeros((0,))
        self.avec2 = np.zeros((0,))
        self.avec3 = np.zeros((0,))
        self.zAlpha = []
        self.wAlpha = []
        self.asize = 0

        self.qv = []
        self.mv = []
        self.iv = []
        self.cvec = []

        self.qvNew = []
        #  self.ratioIntervals = new Interval[0]
        self.columnList = []

        self.fullSolve = False
        self.incrementalPivoting = False
        self.Morig = np.zeros((0, 0))
        self.qorig = np.zeros((0,))
        self.Basis = np.zeros((0, 0))
        self.InvBasis = np.zeros((0, 0))
        self.col = np.zeros((0,))

        self.basisHistory = []
        self.currentBasis = self.BasisState()
        self.proposedBasis = self.BasisState()

        self.zvarNames = None
        self.wvarNames = None

        self.tieBreakHack = -1  # 21


    # @return void
    # @param zNames String[]
    # @param wNames String[]
    def setVariableNames(self, zNames, wNames):
        self.allocateSpace(len(zNames))
        for _i in range(0, zNames.length):
            self.zVars[_i].name = zNames[_i]
        for _i in range(0, wNames.length):
            self.wVars[_i].name = wNames[_i]


    # @return boolean
    # @param basis BasisState
    # @param pcnt int
    def basisHasOccurred(self, basis, pcnt):
        for _i in range(0, pcnt):
            if self.basisHistory[_i].equals(basis):
                return True
        return False

    # @param enable boolean
    def setIncrementalPivoting(self, enable):
        self.incrementalPivoting = enable


    '''
    /**
     * Shows the z variables which are part of the current basis, as well
     * as the non-basic driving variable (in parantheses).
     */
    '''
    # @return String
    # @param driveVar Variable
    def basisString(self, driveVar):
        # StringBuffer sbuf = new StringBuffer(256)
        sbuf = ""
        for _i in range(0, self.msize):
            if self.zVars[_i].isBasic:
                sbuf += " " + self.zVars[_i].getName()

        if self.zVars[self.msize].isBasic:
            sbuf += " z0 ("
            if driveVar is not None:
                sbuf += driveVar.getName()
            else:
                sbuf += "null"
            sbuf += ")"
        return sbuf


    # XXX Need to think better about how to get a "good" epsilon.
    # @return double
    # @param M double[]
    # @param q double[]
    # @param nc int
    def computeEpsilon(self, M, q, nc=None):
        if nc is None:
            nc = len(q)
        # compute the one norm of M
        Mnorm1 = 0.
        for _j in range(0, nc):
            _sum = 0.
            for _i in range(0, nc):
                _sum += abs(M[_i*nc+_j])
            if _sum > Mnorm1:
                Mnorm1 = _sum

        # compute the one norm of q
        qnorm1 = 0.
        for _i in range(0, nc):
            qnorm1 += abs(q[_i])
        maxnorm = max(Mnorm1, qnorm1)
        return 100. * math.sqrt(nc) * maxnorm * self.DOUBLE_PREC

    # @return void
    # @param row VectorNd
    # @param i int
    def getMaaRow(self, row, i_):
        # buf = row.getBuffer()
        buf = row
        for ja in range(0, self.asize):
            _j = self.zAlpha[ja].idx
            if _j == self.msize:
                buf[ja] = self.cvec[i_]
            else:
                buf[ja] = self.Mbuf[i_*self.msize + _j]
        return buf


    # @return void
    # @param col VectorNd
    # @param j int
    def getMaaCol(self, col, j_):
        # buf = col.getBuffer()
        buf = col
        if j_ == self.msize:
            for ia in range(0, self.asize):
                buf[ia] = self.cvec[self.wAlpha[ia].idx]
        else:
            for ia in range(0, self.asize):
                buf[ia] = self.Mbuf[self.wAlpha[ia].idx*self.msize + j_]
        return buf


    # @return double
    # @param i int
    # @param j int
    def getMaaElement(self, i_, j_):
        if j_ == self.msize:
            return self.cvec[i_]
        else:
            return self.Mbuf[i_*self.msize+j_]


    # @return MatrixNd
    def createMaa(self):
        Maa = np.zeros((self.asize, self.asize))
        col = np.zeros((self.asize,))

        for ja in range(0, self.asize):
            col = self.getMaaCol(col, self.zAlpha[ja].idx)
            print ("zAlpha=" + self.zAlpha[ja].getName() + " col=" + self.zAlpha[ja].idx)
            Maa[:, ja] = col

        return Maa


    #TODO:
    # return RaaBuf instead of call-by-reference
    # @return void
    # @param Raa MatrixNd
    # @param Rcol VectorNd
    # @param Rrow VectorNd
    def outerProductUpdate(self, Raa, Rcol, Rrow):
        int w = Raa.getBufferWidth()
        double[] RaaBuf = Raa.getBuffer()
        double[] colBuf = Rcol.getBuffer()
        double[] rowBuf = Rrow.getBuffer()

        for ia in range(0, self.asize):
            for ja in range(0, self.asize):
                RaaBuf[ia*w + ja] -= colBuf[ia]*rowBuf[ja]

    # @return Variable
    # @param dropping Variable
    # @param entering Variable
    def incPivot(self, dropping, entering):

        newDrive = None
        entering.isBasic = True
        dropping.isBasic = False

        newDrive = dropping.complement

        if dropping.isW():

            if entering.isZ():
                # add row and column to Raa 

                #       print ("add row and column")
                Mcol = self.avec0
                Mrow = self.avec1

                col = entering.idx
                row = dropping.idx

                Mcol = self.getMaaCol(Mcol, col)
                Mrow = self.getMaaRow(Mrow, row)

                Rvec = self.avec2
                Rrow = self.avec3

                self.Raa.mul(Rvec, Mcol)
                Rrc = 1/(self.getMaaElement(row, col) - Mrow.dot(Rvec))
                self.Raa.mulTranspose(Rrow, Mrow)
                Rrow.scale(-Rrc)
                self.Raa.setSize(self.asize+1, self.asize+1)

                self.outerProductUpdate(self.Raa, Rvec, Rrow)

                for ia in range(0, self.asize):
                    self.Raa.set(ia, self.asize, -Rrc*Rvec.get(ia))
                    self.Raa.set(self.asize, ia, Rrow.get(ia))
                self.Raa.set(self.asize, self.asize, Rrc)

                self.avec0 = np.zeros((self.asize+1,))
                self.avec1 = np.zeros((self.asize+1,))
                self.avec2 = np.zeros((self.asize+1,))
                self.avec3 = np.zeros((self.asize+1,))

                self.zAlpha[self.asize] = entering
                entering.col = self.asize
                self.wAlpha[self.asize] = dropping
                dropping.col = self.asize
                self.asize += 1
            else:
                # replace row in Maa

                #       print ("replace row")

                oldRow = entering.idx
                newRow = dropping.idx

                MrowOld = self.avec0
                MrowDel = self.avec1

                MrowOld = self.getMaaRow(MrowOld, oldRow)
                MrowDel = self.getMaaRow(MrowDel, newRow)
                MrowDel.sub(MrowOld)

                Rcol = self.avec2
                Rvec = self.avec3

                int col = entering.col
                Raa.getColumn(col, Rcol)
                Raa.mulTranspose(Rvec, MrowDel)
                Rvec.scale(1/(1+MrowDel.dot(Rcol)))

                outerProductUpdate(Raa, Rcol, Rvec)

                wAlpha[col] = dropping
                dropping.col = col
        else:
            if entering.isW():
                # delete row and column from Maa
                int oldRow = dropping.col
                int oldCol = entering.col

                #       print ("delete row and column")

                #       print ("oldRow=" + oldRow)
                #       print ("oldCol=" + oldCol)

                #       print ("RaaOld=[\n" + 
                #                   Raa.toString("%10.6f") + "]")

                #       MatrixNd Maa= createMaa()
                #       print ("MaaOld=[\n" + 
                #                   Maa.toString("%10.6f") + "]")

                #       print ("dropping " + dropping.getName())
                #       print ("entering " + entering.getName())

                #       for ia in range(0, asize):
                #        {
                #           print (zAlpha[ia].getName() + " " + zAlpha[ia].col)
                #        }
                #       for ia in range(0, asize):
                #        {
                #           print (wAlpha[ia].getName() + " " +  wAlpha[ia].col)
                #        }

                VectorNd Rrow = avec0
                VectorNd Rcol = avec1

                Raa.getRow(oldRow, Rrow)
                Raa.getColumn(oldCol, Rcol)
                double Rrc = Raa.get(oldRow, oldCol)

                int w = Raa.getBufferWidth()
                double[] RaaBuf = Raa.getBuffer()

                for ia in range(oldRow, asize-1):
                    zAlpha[ia] = zAlpha[ia+1]
                    zAlpha[ia].col = ia
                    Rcol.set(ia, Rcol.get(ia+1))
                    for ja in range(0, asize):
                        RaaBuf[ia*w+ja] = RaaBuf[(ia+1)*w+ja]
                for ja in range(oldCol, asize-1):
                    wAlpha[ja] = wAlpha[ja+1]
                    wAlpha[ja].col = ja
                    Rrow.set(ja, Rrow.get(ja+1))
                    for ia in range(0, asize-1):
                        RaaBuf[ia*w+ja] = RaaBuf[ia*w+ja+1]
                Raa.setSize(asize-1, asize-1)
                Rcol.setSize(asize-1)
                Rrow.setSize(asize-1)

                #           print ("Raa reduced=\n" + Raa.toString("%10.6f"))

                #       print ("Rcol=" + Rcol.toString("%10.6f"))
                #       print ("Rrow=" + Rrow.toString("%10.6f"))
                #       print ("Rrc=" + Rrc)

                Rcol.scale(1/Rrc)

                outerProductUpdate(Raa, Rcol, Rrow)

                #       print ("RaaNew=[\n" +
                #                   Raa.toString("%10.6f") + "]")

                avec2.setSize(asize-1)
                avec3.setSize(asize-1)
                asize--

                #       Maa= createMaa()
                #       print ("MaaNew=[\n" +
                #                   Maa.toString("%10.6f") + "]")
            else:
                # replace column in Maa

                #       print ("replace column")

                oldCol = dropping.idx
                newCol = entering.idx

                VectorNd McolOld = avec0
                VectorNd McolDel = avec1

                McolOld = self.getMaaCol(McolOld, oldCol)
                McolDel = self.getMaaCol(McolDel, newCol)
                McolDel.sub(McolOld)

                VectorNd Rrow = avec2
                VectorNd Rvec = avec3

                int col = dropping.col
                Raa.getRow(col, Rrow)
                Raa.mul(Rvec, McolDel)

                Rrow.scale(1/(1+Rrow.dot(McolDel)))

                outerProductUpdate(Raa, Rvec, Rrow)

                zAlpha[col] = entering
                entering.col = col
        if newDrive is not null:
            if newDrive.isZ():
                self.wzVar = newDrive.complement
            else:
                self.wzVar = newDrive
        else:
            self.wzVar = None
        self.computeQv(self.qv)
        self.cumulativePivotCnt += 1
        return newDrive

    # @return boolean
    # @param minCond double
    def computeMvFromBasis(self, minCond):
        nr = self.msize
        nc = self.msize+1

        double[] MvNew = new double[nr*nc]
        double[] qvNew = new double[nr]

        for j in range(0, nr):
            Variable bvar = basicVars[j]
            if bvar.isZ0():
                Morig.getColumn(nr, col)
                col.negate()
            elif bvar.isZ():
                Morig.getColumn(bvar.idx, col)
                col.negate()
            else:
                col.setZero()
                col.set(bvar.idx, 1)
            Basis.setColumn(j, col)
        LUDecomposition LU = new LUDecomposition(nr)
        LU.set(Basis)
        double rcond = 1/LU.conditionEstimate(Basis)
        if rcond < minCond:
            return False

        LU.inverse(InvBasis)
        col.mul(InvBasis, qorig)
        col.get(qvNew)
        for j in range(0, nr+1):
            Variable nbvar = nonBasicVars[j]
            if nbvar.isZ0():
                # Morig.getColumn(nr, col)
                for i in range(0, nr):
                    col.set(i, cvec[i])
                col.mul(InvBasis, col)
                #       print("col:")
                #       for i in range(0, nr):
                #        {
                # print (col.get(i))
                #        }
            elif nbvar.isZ():
                Morig.getColumn(nbvar.idx, col)
                col.mul(InvBasis, col)
            else:
                InvBasis.getColumn(nbvar.idx, col)
                col.negate()
            for i in range(0, nr):
                MvNew[i*nc+j] = col.get(i)

        for i in range(0, nr):
            for j in range(0, nc):
                Mv[i*nc+j] = MvNew[i*nc+j]
            qv[i] = qvNew[i]
        return True

    # @return Variable
    # @param dropping Variable
    # @param entering Varialbe
    def pivot(self, dropping, entering):
        # transform Mv and qv to reflect and exchange of variables

        if self.incrementalPivoting:
            return self.incPivot(dropping, entering)

        int nr = msize
        int nc = msize+1

        int s = dropping.col
        int r = entering.col

        double q_s = qv[s]
        double m_sr = Mv[s*nc+r]

        #     print ("m_sr=" + m_sr)

        # update qv
        for i in range(0, nr):
            if i == s:
                qv[i] = -q_s/m_sr
            else:
                qv[i] = qv[i] - q_s*(Mv[i*nc+r]/m_sr)

        # update Mv(i,j) for i != s and j != r
        for i in range(0, nr):
            if i != s:
                double m_ir = Mv[i*nc+r]
                for j in range(0, nc):
                    if j != r:
                        Mv[i*nc+j] -= (m_ir/m_sr)*Mv[s*nc+j]
        # update Mv(s,j)
        for j in range(0, nc):
            if j != r:
                Mv[s*nc+j] /= -m_sr
        # update Mv(i,r)
        for i in range(0, nr):
            if i != s:
                Mv[i*nc+r] /= m_sr
        Mv[s*nc+r] = 1/m_sr

        # update variable listings

        #     Variable entering = nonBasicVars[r]
        #     Variable dropping = basicVars[s]

        #     if entering.col != r:
        #      {
        # print ("entering=" + entering.getName())
        #        print ("col=" + entering.col + " r=" + r)
        #        System.exit(1)
        #      }
        #     if dropping.col != s:
        #      {
        # print ("dropping=" + dropping.getName())
        #        print ("col=" + dropping.col + " s=" + s)
        #        System.exit(1)
        #      }

        entering.isBasic = True
        dropping.col = r
        dropping.isBasic = False
        entering.col = s

        self.nonBasicVars[r] = dropping

        self.basicVars[s] = entering
        nonCompIdx = dropping.idx

        if self.fullSolve:
            self.computeMvFromBasis(0)

        self.cumulativePivotCnt += 1
        Variable newDrive = dropping.complement
        if newDrive != null:
            if newDrive.isZ():
                self.wzVar = newDrive.complement
            else:
                self.wzVar = newDrive
        else:
            self.wzVar = None
        return newDrive

    # @return void
    # @param n int
    def allocateSpace(self, n):

        super.allocateSpace(n)

        if len(self.qv) < n:

            # Mv = new double[n*(2*n+1)]
            Mv = new double[n*(n+1)]
            qv = new double[n]
            mv = new double[n]
            iv = new double[n]
            Raa = new MatrixNd(n, n)
            avec0 = new VectorNd(n)
            avec1 = new VectorNd(n)
            avec2 = new VectorNd(n)
            avec3 = new VectorNd(n)
            cvec = new double[n]
            for i in range(0, n):
                cvec[i] = 1
            qvNew = new double[n]
            basicVars = new Variable[n]
            nonBasicVars = new Variable[n+1]

            zAlpha = new Variable[n]
            wAlpha = new Variable[n]

            Class varClass = new Variable().getClass()
            oldLength

            oldLength = wVars.length
            wVars = (Variable[])growObjectArray(wVars, n, varClass)
            for i in range(oldLength, wVars.length):
                wVars[i].setIndex(i)
            oldLength = zVars.length
            zVars = (Variable[])growObjectArray(zVars, n+1, varClass)
            for i in range(oldLength, zVars.length):
                zVars[i].setIndex(i)

            #        BasicNode[] oldNodeList = nodeList
            #        nodeList = new BasicNode[n]
            #        for i in range(0, oldNodeList.length):
            #         {
            # nodeList[i] = oldNodeList[i]
            #         }
            #        for i in range(oldNodeList.length, nodeList.length):
            #         {
            # nodeList[i] = new BasicNode (i)
            #         }

            if cycleCheckingEnabled and (basisHistory.length == 0):

                basisHistory = new BasisState[maxBasisHistory]
                for i in range(0, basisHistory.length):
                    basisHistory[i] = new BasisState()
            # updateNodeAllocation (n)


    '''
    @return void
    @param w double[]
    @param z double[]
    @param M double[]
    @param q double[]
    @param n int
    @param eps double
    '''
    def checkArgs(self, w, z, M, q, n, eps):
        if M.length < n*n:
            throw new IllegalArgumentException("Incompatible size for matrix M")
        if q.length < n:
            throw new IllegalArgumentException("Incompatible size for vector q")
        if z != null && z.length < n:
            throw new IllegalArgumentException("Vector z too small")
        if w != null && w.length < n:
            throw new IllegalArgumentException("Vector w too small")

        # compute an epsilon
        if eps == AUTO_EPSILON:
            epsilon = computeEpsilon(M, q, n)
        else:
            epsilon = eps
        self.allocateSpace(n)


    '''
    @return void
    @param w double[]
    @param z double[]
    @param M double[]
    @param q double[]
    @param qv double[]
    @param n int
    '''
    def computeSolution(self, w, z, M, q, qv, n):
        if (w is not None) or (z is not None):
            if z is None:
                z = [0.]*n
            for i in range(0, n):
                z[i] = 0
            if self.incrementalPivoting:
                for i in range(0, n):
                    if self.zVars[i].isBasic:
                        z[self.zVars[i].idx] = qv[self.zVars[i].idx]
            else:
                for i in range(0, n):
                    if self.basicVars[i].isZ():
                        z[self.basicVars[i].idx] = qv[i]
            for i in range(0, n):
                wx = q[i]
                for j in range(0, n):
                    wx += M[i*n+j]*z[j]
                w[i] = wx



    # @return void
    # @param y double[]
    # @param var Variable
    def computeMvCol(self, y, var):
        wVars = self.wVars
        msize = self.msize
        asize = self.asize
        avec0 = self.avec0
        avec1 = self.avec1
        avec2 = self.avec2
        zAlpha = self.zAlpha

        if not self.incrementalPivoting:
            # int nc = 2*msize+1
            nc = msize+1
            for i in range(0, msize):
                y[i] = self.Mv[i*nc+var.col]
        elif var.isZ():
            MabCol = avec0
            Rprod = avec1
            MbaRow = avec2

            col = var.idx

            MabCol = self.getMaaCol(MabCol, col)
            self.Raa.mul(Rprod, MabCol)
            for ia in range(0, asize):
                if zAlpha[ia] == self.z0Var:
                    idx = self.wzVar.idx
                else:
                    idx = zAlpha[ia].idx
                y[idx] = -Rprod.get(ia)
            for i in range(0, msize):
                if wVars[i].isBasic:
                    row = wVars[i].idx
                    MbaRow = self.getMaaRow(MbaRow, row)
                    y[row] = self.Mbuf[row*msize+col] - Rprod.dot(MbaRow)
        else:  # var.isW()
            Rprod = avec0
            MbaRow = avec1

            self.Raa.getColumn(var.col, Rprod)
            for ia in range(0, asize):
                if zAlpha[ia] == self.z0Var:
                    idx = self.wzVar.idx
                else:
                    idx = zAlpha[ia].idx
                y[idx] = Rprod.get(ia)

            for i in range(0, msize):
                if wVars[i].isBasic:
                    row = wVars[i].idx
                    MbaRow = self.getMaaRow(MbaRow, row)
                    y[row] = Rprod.dot(MbaRow)


    # @return void
    # @param y double[]
    def computeQv(self, y):
        msize = self.msize
        asize = self.asize
        if not self.incrementalPivoting:
            if y != self.qv:
                for i in range(0, msize):
                    y[i] = self.qv[i]
        else:
            qa = self.avec0
            Rprod = self.avec1
            MbaRow = self.avec2

            for ia in range(0, asize):
                qa.set(ia, self.qbuf[self.wAlpha[ia].idx])
            self.Raa.mul(Rprod, qa)
            for ia in range(0, asize):
                if self.zAlpha[ia] == self.z0Var:
                    idx = self.wzVar.idx
                else:
                    idx = self.zAlpha[ia].idx

                y[idx] = -Rprod.get(ia)
            for i in range(0, msize):
                if self.wVars[i].isBasic:
                    row = self.wVars[i].idx
                    MbaRow = self.getMaaRow(MbaRow, row)
                    y[row] = self.qbuf[row] - Rprod.dot(MbaRow)

    # @return Variable
    # @param idx int
    def getBasic(self, idx):
        if self.incrementalPivoting:
            if self.zVars[idx].isBasic:
                return self.zVars[idx]
            else:
                return self.wVars[idx]
        else:
            return self.basicVars[idx]


    '''
    @return int
    @param w double vector
    @param z double vector
    @param M double matrix
    @param q double vector
    @param nr int dimension
    @param eps double epsilon
    @param preset boolean preset
    '''
    def solve(self, w, z, M, q, nr=None, eps=None, preset=None):
        if nr is None:
            nr = len(q)
        if eps is None:
            eps = self.AUTO_EPSILON
        asize = self.asize
        driveVar = self.Variable()
        blockingVar = self.Variable()
        numCand = 0
        origNumCand = 0
        basisWasPreset = False

        nc = nr+1
        cycleCheck = False

        # print ("problem size = " + nr)
        if self.cycleCheckingEnabled:
            if nr <= self.maxCycleCheckSize:
                cycleCheck = True
            else:
                print ("Warning: no cycle check because problem size exceeds " + self.maxCycleCheckSize)

        self.checkArgs(w, z, M, q, nr, eps)

        if (self.fullSolve or preset) is not None:
            Morig.setSize(nr, nr+1)
            for i in range(0, nr):
                for j in range(0, nr):
                    Morig.set(i, j, M[i*nr+j])

                Morig.set(i, nr, 1)

            qorig.setSize(nr)
            qorig.set(q)
            Basis.setSize(nr, nr)
            InvBasis.setSize(nr, nr)
            col.setSize(nr)

        qmin = Double.POSITIVE_INFINITY
        pivotCnt = 0
        maxPivotCnt = 1000  # nr*nr

        Mbuf = M
        qbuf = q
        msize = nr

        if not self.incrementalPivoting:
            for i in range(0, nr):
                for j in range(0, nr):
                    Mv[i*nc+j] = M[i*nr+j]
                    Mv[i*nc+nr] = cvec[i]
            #  for j in range(nr+1, 2*nr+1):
            #   {
            # set the last nr columns to an nrXnr identity
            # Mv[i*nc+j] = (i == j-(nr+1)) ? 1 : 0
            #   }
            qv[i] = q[i]
        else:
            for i in range(0, msize):
                qv[i] = q[i]
            self.asize = 0
            asize = self.asize
            Raa.setSize(asize, asize)
            avec0.setSize(asize)
            avec1.setSize(asize)
            avec2.setSize(asize)
            avec3.setSize(asize)

        for i in range(0, nr):
            Variable wvar = wVars[i]
            Variable zvar = zVars[i]

            wvar.init(W_VAR, zvar)
            zvar.init(Z_VAR, wvar)

            basicVars[i] = wvar
            nonBasicVars[i] = zvar

            qv[i] = q[i]

        z0Var = zVars[nr]
        z0Var.init(Z0, null)

        nonBasicVars[nr] = z0Var
        nonCompIdx = nr

        if preset is not None:

            numActive = 0
            # initialize cover vector
            for i in range(0, nr):
                cvec[i] = 0

            for i in range(0, min(preset.length, nr)):
                if preset[i]:
                    zVars[i].isBasic = True
                    wVars[i].isBasic = False

                    basicVars[i] = zVars[i]
                    nonBasicVars[i] = wVars[i]
                    numActive += 1
                    
            for j in range(0, nr):
                if zVars[j].isBasic:
                    for i in range(0, nr):
                        cvec[i] -= M[i*nr+j]
                else:
                    cvec[j] += 3

            if numActive > 0:
             
                if not computeMvFromBasis(1000*DOUBLE_PREC):
                    print ("pre-basis ill-conditioned; ignoring ...")
                    for i in range(0, nr):
                        zVars[i].isBasic = False
                        wVars[i].isBasic = True
                        cvec[i] = 1

                        basicVars[i] = wVars[i]
                        nonBasicVars[i] = zVars[i]
                else:
                    basisWasPreset = True
            
        else:
            for i in range(0, nr):
                cvec[i] = 1
        
        if (debug & SHOW_MINRATIO) != 0:
            print ("Initial qv:")
            NumberFormat fmt = new NumberFormat("%14.8f")
            for i in range(0, nr):
                print (basicVars[i].getName() + " " +fmt.format(qv[i]))

        for i in range(0, nr):
            if basisWasPreset:
                mv[i] = wVars[i].isBasic ? 3 : 1
            else:
                mv[i] = cvec[i]

        for i in range(0, nr):
            mv[i] = -mv[i]

        int imin = lexicoMinRatioTest(mv, qv, nr, -1, True)
        if imin == -1:
            # then q >= 0 and so z = 0 is a solution
            self.computeSolution(w, z, M, q, qv, nr)
            # print ("basis: " + basisString(null))
            return self.SOLVED
        
        '''
        #     if preset != null:
        #      {
         for i in range(0, nr):
        #         {
            if zVars[i].isBasic:
        #        {
           Mv[i*nc+nr] = 1
        #        }
        #       else:
        #        {
           Mv[i*nc+nr] = 2; 
        #        }
        #         }
        #      }
        '''    
        # zx to be the smallest zx s.t. w = q + d zx >= 0
        # double zx = qmin
        r = imin
        i_zx = r

        if (self.debug & self.SHOW_MINRATIO) != 0:
            print ("Initial:")
        

        if wVars[r].isBasic:
            driveVar = pivot(wVars[r], z0Var)
        else:
            driveVar = pivot(zVars[r], z0Var)

        if cycleCheck:
            currentBasis.init(driveVar)

        # CandidateList listCopy = null

        while pivotCnt < maxPivotCnt:
            # driving variable is indexed by r.
            # find the blocking variable from the r-th column
            computeMvCol(mv, driveVar)

            if (debug & (SHOW_MINRATIO | SHOW_BASIS)) != 0:
                print ("Basis: " + basisString(driveVar))
            
            if (debug & SHOW_MINRATIO) != 0:
                print ("epsilon=" + epsilon)
                print ("driving variable="+ driveVar.getName())
            
            int z_i = incrementalPivoting ? wzVar.idx : i_zx
            int s = lexicoMinRatioTest(mv, qv, nr, z_i, /*initial=*/False)
            if s == -1:
                # then driving variable is unblocked, so we can't find a solution
                print ("unbounded ray")
                return UNBOUNDED_RAY
            
            elif s == z_i:
                # zx blocks the driving variable
                pivot(z0Var, driveVar)
                if (debug & SHOW_MINRATIO) != 0:
                    printQv("Final:", qv, nr)
                computeSolution(w, z, M, q, qv, nr)
                # print ("basis: " + basisString(null))
                if preset != null:
                    for i in range(0, min(preset.length, nr)):
                        preset[i] = zVars[i].isBasic
                return SOLVED
            else:
                driveVar = pivot(getBasic(s), driveVar)
            pivotCnt++

        return CYCLING_DETECTED

    # @return double
    def getEpsilon(self):
        return epsilon

    # @return boolean
    # @param j int
    def wIsBasic(self, j):
        return self.wVars[j].isBasic

    # @return void
    def getBasisColumn(self, iv, j):
        self.computeMvCol(iv, self.wVars[j])

    # @return Variable[]
    def getBasicVars(self):
        if self.incrementalPivoting:
            _vars = [self.Variable]*self.msize

            for i in range(0, self.msize):
                if self.wVars[i].isBasic:
                    _vars[i] = self.wVars[i]
                elif self.zVars[i].isBasic:
                    _vars[i] = self.zVars[i]
                else:
                    _vars[i] = self.z0Var
            return _vars
        else:
            return self.basicVars

    # @return Variable
    def getWzVar(self):
        return self.wzVar

    # @return String
    def getBasisString(self):
        return self.basisString(None)
