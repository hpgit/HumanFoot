import LemkeSolverBase


class LemkeLCPSolver(LemkeSolverBase):
    '''
    #  private Variable[] variableList = new Variable[0]
    private Variable[] zVars = new Variable[0]
    private Variable[] wVars = new Variable[0]
    private Variable[] basicVars
    private Variable[] nonBasicVars
    private Variable z0Var
    private Variable wzVar
    private int nonCompIdx = -1

    private double[] Mv



    # stuff for incremental pivoting

    private double[] Mbuf
    private double[] qbuf
    private int msize
    private MatrixNd Raa = new MatrixNd(0,0)
    #  private double[] RaaBuf = new double[0]
    private VectorNd avec0 = new VectorNd(0)
    private VectorNd avec1 = new VectorNd(0)
    private VectorNd avec2 = new VectorNd(0)
    private VectorNd avec3 = new VectorNd(0)
    private Variable[] zAlpha = new Variable[0]
    private Variable[] wAlpha = new Variable[0]
    private int asize = 0

    private double[] qv = new double[0]
    private double[] mv = new double[0]
    private double[] iv = new double[0]
    private double[] cvec = new double[0]

    private double[] qvNew = new double[0]
    #  private Interval[] ratioIntervals = new Interval[0]
    private int[] columnList = new int[0]

    private boolean fullSolve = false
    private boolean incrementalPivoting = false
    private MatrixNd Morig = new MatrixNd(0,0)
    private VectorNd qorig = new VectorNd(0)
    private MatrixNd Basis = new MatrixNd(0,0)
    private MatrixNd InvBasis = new MatrixNd(0,0)
    private VectorNd col = new VectorNd(0)

    private BasisState[] basisHistory = new BasisState[0]
    private BasisState currentBasis = new BasisState()
    private BasisState proposedBasis = new BasisState()

    private String[] zvarNames = null
    private String[] wvarNames = null
    '''
    def __init__():
        #  private Variable[] variableList = new Variable[0]
        private Variable[] zVars = new Variable[0]
        private Variable[] wVars = new Variable[0]
        private Variable[] basicVars
        private Variable[] nonBasicVars
        private Variable z0Var
        private Variable wzVar
        private int nonCompIdx = -1

        private double[] Mv


        # stuff for incremental pivoting
        private double[] Mbuf
        private double[] qbuf
        private int msize
        private MatrixNd Raa = new MatrixNd(0,0)
        #  private double[] RaaBuf = new double[0]
        private VectorNd avec0 = new VectorNd(0)
        private VectorNd avec1 = new VectorNd(0)
        private VectorNd avec2 = new VectorNd(0)
        private VectorNd avec3 = new VectorNd(0)
        private Variable[] zAlpha = new Variable[0]
        private Variable[] wAlpha = new Variable[0]
        private int asize = 0

        private double[] qv = new double[0]
        private double[] mv = new double[0]
        private double[] iv = new double[0]
        private double[] cvec = new double[0]

        private double[] qvNew = new double[0]
        #  private Interval[] ratioIntervals = new Interval[0]
        private int[] columnList = new int[0]

        private boolean fullSolve = false
        private boolean incrementalPivoting = false
        private MatrixNd Morig = new MatrixNd(0,0)
        private VectorNd qorig = new VectorNd(0)
        private MatrixNd Basis = new MatrixNd(0,0)
        private MatrixNd InvBasis = new MatrixNd(0,0)
        private VectorNd col = new VectorNd(0)

        private BasisState[] basisHistory = new BasisState[0]
        private BasisState currentBasis = new BasisState()
        private BasisState proposedBasis = new BasisState()

        private String[] zvarNames = null
        private String[] wvarNames = null

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
        #         if ((zactive & (1L<<i)) != 0):
        #             sbuf.append (" z")
        #             sbuf.append ((i+1))
        #     if (driveVar != null):
        #         sbuf.append (" z0 (" + driveVar.getName() + ")")
        #     return sbuf.toString()

    # @return void
    # @param zNames String[]
    # @param wNames String[]
    def setVariableNames(zNames, wNames):
        allocateSpace(zNames.length)
        for i in range(0, zNames.length):
            zVars[i].name = zNames[i]
        for i in range(0, wNames.length):
            wVars[i].name = wNames[i]


    # @return boolean
    # @param basis BasisState
    # @param pcnt int
    def basisHasOccurred(basis, pcnt):
        for i in range(0, pcnt):
            if basisHistory[i].equals(basis):
                return True
        return False

    # @param enable boolean
    def setIncrementalPivoting(enable):
        incrementalPivoting = enable


    '''
    /**
     * Shows the z variables which are part of the current basis, as well
     * as the non-basic driving variable (in parantheses).
     */
    '''
    # @return String
    # @param driveVar Variable
    def basisString(driveVar):
        StringBuffer sbuf = new StringBuffer(256)
        for i in range(0, msize):
            if (zVars[i].isBasic):
                sbuf.append(" ")
                sbuf.append(zVars[i].getName())

        if (zVars[msize].isBasic):
            sbuf.append(" z0 (")
            if (driveVar != null):
                sbuf.append(driveVar.getName())
            else:
                sbuf.append("null")
            sbuf.append(")")
        return sbuf.toString()


    # @return double
    # @param M double[]
    # @param q double[]
    def computeEpsilon(M, q):
        return computeEpsilon(M, q, q.length)


    # XXX Need to think better about how to get a "good" epsilon.
    # @return double
    # @param M double[]
    # @param q double[]
    # @param nc int
    def computeEpsilon(M, q, nc):
        # compute the one norm of M
        double Mnorm1 = 0
        for j in range(0, nc):
            double sum = 0
            for i in range(0, nc):
                sum += Math.abs(M[i*nc+j])
            if (sum > Mnorm1):
                Mnorm1 = sum

        # compute the one norm of q
        double qnorm1 = 0
        for i in range(0, nc):
            qnorm1 += Math.abs(q[i])
        double maxnorm = Math.max(Mnorm1, qnorm1)
        return 100*Math.sqrt(nc)*maxnorm*DOUBLE_PREC

    # @return void
    # @param row VectorNd
    # @param i int
    def getMaaRow(row, i):
        double[] buf = row.getBuffer()
        for ja in range(0, asize):
            int j = zAlpha[ja].idx
            if (j == msize):
                buf[ja] = cvec[i]
            else:
                buf[ja] = Mbuf[i*msize + j]


    # @return void
    # @param col VectorNd
    # @param j int
    def getMaaCol(col, j):
        double[] buf = col.getBuffer()
        if (j == msize):
            for ia in range(0, asize):
                buf[ia] = cvec[wAlpha[ia].idx]
        else:
            for ia in range(0, asize):
                buf[ia] = Mbuf[wAlpha[ia].idx*msize+j]


    # @return double
    # @param i int
    # @param j int
    def getMaaElement(i, j):
        if (j == msize):
            return cvec[i]
        else:
            return Mbuf[i*msize+j]


    # @return MatrixNd
    def createMaa():
        MatrixNd Maa = new MatrixNd(asize, asize)
        VectorNd col = new VectorNd(asize)
        for ja in range(0, asize):

            getMaaCol(col, zAlpha[ja].idx)
            print ("zAlpha=" + zAlpha[ja].getName() + " col=" + zAlpha[ja].idx)
            Maa.setColumn(ja, col)

        return Maa


    # @return void
    # @param Raa MatrixNd
    # @param Rcol VectorNd
    # @param Rrow VectorNd
    def outerProductUpdate(Raa, Rcol, Rrow):
        int w = Raa.getBufferWidth()
        double[] RaaBuf = Raa.getBuffer()
        double[] colBuf = Rcol.getBuffer()
        double[] rowBuf = Rrow.getBuffer()

        for ia in range(0, asize):
            for ja in range(0, asize):
                RaaBuf[ia*w + ja] -= colBuf[ia]*rowBuf[ja]

    # @return Variable
    # @param dropping Variable
    # @param entering Variable
    def incPivot(dropping, entering):

        Variable newDrive = null
        entering.isBasic = true
        dropping.isBasic = false

        newDrive = dropping.complement

        if dropping.isW():

            if entering.isZ():
                # add row and column to Raa 

                #       print ("add row and column")
                VectorNd Mcol = avec0
                VectorNd Mrow = avec1

                int col = entering.idx
                int row = dropping.idx

                getMaaCol(Mcol, col)
                getMaaRow(Mrow, row)

                VectorNd Rvec = avec2
                VectorNd Rrow = avec3

                Raa.mul(Rvec, Mcol)
                double Rrc = 1/(getMaaElement(row, col) - Mrow.dot(Rvec))
                Raa.mulTranspose(Rrow, Mrow)
                Rrow.scale(-Rrc)
                Raa.setSize(asize+1, asize+1)

                outerProductUpdate(Raa, Rvec, Rrow)

                for ia in range(0, asize):
                    Raa.set(ia, asize, -Rrc*Rvec.get(ia))
                    Raa.set(asize, ia, Rrow.get(ia))
                Raa.set(asize, asize, Rrc)

                avec0.setSize(asize+1)
                avec1.setSize(asize+1)
                avec2.setSize(asize+1)
                avec3.setSize(asize+1)

                zAlpha[asize] = entering
                entering.col = asize
                wAlpha[asize] = dropping
                dropping.col = asize
                asize++
            else:
                # replace row in Maa

                #       print ("replace row")

                int oldRow = entering.idx
                int newRow = dropping.idx

                VectorNd MrowOld = avec0
                VectorNd MrowDel = avec1

                getMaaRow(MrowOld, oldRow)
                getMaaRow(MrowDel, newRow)
                MrowDel.sub(MrowOld)

                VectorNd Rcol = avec2
                VectorNd Rvec = avec3

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

                int oldCol = dropping.idx
                int newCol = entering.idx

                VectorNd McolOld = avec0
                VectorNd McolDel = avec1

                getMaaCol(McolOld, oldCol)
                getMaaCol(McolDel, newCol)
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
                wzVar = newDrive.complement
            else:
                wzVar = newDrive
        else:
            wzVar = null
        computeQv(qv)
        cumulativePivotCnt++
        return newDrive

    # @return boolean
    # @param minCond double
    def computeMvFromBasis(minCond):
        int nr = msize
        int nc = msize+1

        double[] MvNew = new double[nr*nc]
        double[] qvNew = new double[nr]

        for j in range(0, nr):
            Variable bvar = basicVars[j]
            if bvar.isZ0():
                Morig.getColumn(nr, col)
                col.negate()
            else if bvar.isZ():
                Morig.getColumn(bvar.idx, col)
                col.negate()
            else:
                col.setZero()
                col.set(bvar.idx, 1)
            Basis.setColumn(j, col)
        LUDecomposition LU = new LUDecomposition(nr)
        LU.set(Basis)
        double rcond = 1/LU.conditionEstimate(Basis)
        if (rcond < minCond):
            return false

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
            else if nbvar.isZ():
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
        return true

    # @return Variable
    # @param dropping Variable
    # @param entering Varialbe
    def pivot(dropping, entering):
        # transform Mv and qv to reflect and exchange of variables

        if (incrementalPivoting):
            return incPivot(dropping, entering)

        int nr = msize
        int nc = msize+1

        int s = dropping.col
        int r = entering.col

        double q_s = qv[s]
        double m_sr = Mv[s*nc+r]

        #     print ("m_sr=" + m_sr)

        # update qv
        for i in range(0, nr):
            if (i == s):
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

        #     if (entering.col != r):
        #      {
        # print ("entering=" + entering.getName())
        #        print ("col=" + entering.col + " r=" + r)
        #        System.exit(1)
        #      }
        #     if (dropping.col != s):
        #      {
        # print ("dropping=" + dropping.getName())
        #        print ("col=" + dropping.col + " s=" + s)
        #        System.exit(1)
        #      }

        entering.isBasic = true
        dropping.col = r
        dropping.isBasic = false
        entering.col = s

        nonBasicVars[r] = dropping

        basicVars[s] = entering
        nonCompIdx = dropping.idx

        if fullSolve:
            computeMvFromBasis(0)

        cumulativePivotCnt++
        Variable newDrive = dropping.complement
        if (newDrive != null):
            wzVar = newDrive.isZ() ? newDrive.complement : newDrive
        else:
            wzVar = null
        return newDrive

    # @return void
    # @param n int
    def allocateSpace(n):

        super.allocateSpace (n)

        if (qv.length < n):

            # Mv = new double[n*(2*n+1)]
            Mv = new double[n*(n+1)]
            qv = new double[n]
            mv = new double[n]
            iv = new double[n]
            Raa = new MatrixNd (n, n)
            avec0 = new VectorNd (n)
            avec1 = new VectorNd (n)
            avec2 = new VectorNd (n)
            avec3 = new VectorNd (n)
            cvec = new double[n]
            for i in range(0, n):
                cvec[i] = 1; 
            qvNew = new double[n]
            basicVars = new Variable[n]
            nonBasicVars = new Variable[n+1]

            zAlpha = new Variable[n]
            wAlpha = new Variable[n]

            Class varClass = new Variable().getClass()
            int oldLength

            oldLength = wVars.length
            wVars = (Variable[])growObjectArray(wVars, n, varClass)
            for i in range(oldLength, wVars.length):
                wVars[i].setIndex (i); 
            oldLength = zVars.length
            zVars = (Variable[])growObjectArray(zVars, n+1, varClass)
            for i in range(oldLength, zVars.length):
                zVars[i].setIndex (i); 

            #        BasicNode[] oldNodeList = nodeList
            #        nodeList = new BasicNode[n]
            #        for i in range(0, oldNodeList.length):
            #         {
            #nodeList[i] = oldNodeList[i]
            #         }
            #        for i in range(oldNodeList.length, nodeList.length):
            #         {
            #nodeList[i] = new BasicNode (i)
            #         }

            if (cycleCheckingEnabled && basisHistory.length == 0):

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
    def checkArgs(w, z, M, q, n, eps):
        if (M.length < n*n):
            throw new IllegalArgumentException ("Incompatible size for matrix M"):
        if (q.length < n):
            throw new IllegalArgumentException ("Incompatible size for vector q"):
        if (z != null && z.length < n):
            throw new IllegalArgumentException ("Vector z too small")
        if (w != null && w.length < n):
            throw new IllegalArgumentException ("Vector w too small")

        # compute an epsilon
        if (eps == AUTO_EPSILON):
            epsilon = computeEpsilon (M, q, n)
        else:
            epsilon = eps          
        allocateSpace (n)

    '''
    @return void
    @param w double[]
    @param z double[]
    @param M double[]
    @param q double[]
    @param qv double[]
    @param n int
    '''
    def computeSolution(w, z, M, q, qv, n):
        if (w != null || z != null):
            if (z == null):
                z = new double[n]
            for i in range(0, n):
                z[i] = 0
            if (incrementalPivoting):
                for i in range(0, n):
                    if (zVars[i].isBasic):
                        z[zVars[i].idx] = qv[zVars[i].idx]
            else:
                for i in range(0, n):
                    if (basicVars[i].isZ()):
                        z[basicVars[i].idx] = qv[i] 
            for i in range(0, n):
                double wx = q[i]
                for j in range(0, n):
                    wx += M[i*n+j]*z[j]
                w[i] = wx

    
    # @return int
    # @param w double[]
    # @param z double[]
    # @param M double[]
    # @param q double[]
    def solve(w, z, M, q):
        return solve (w, z, M, q, q.length, AUTO_EPSILON)

    private int tieBreakHack = -1; # 21


    # @return void
    # @param y double[]
    # @param var Variable
    def computeMvCol(y, var):
        if (!incrementalPivoting):
            # int nc = 2*msize+1
            int nc = msize+1
            for i in range(0, msize):
                y[i] = Mv[i*nc+var.col];  
        else if (var.isZ()):
            VectorNd MabCol = avec0
            VectorNd Rprod = avec1
            VectorNd MbaRow = avec2

            int col = var.idx

            getMaaCol (MabCol, col)
            Raa.mul (Rprod, MabCol)
            for ia in range(0, asize):
                int idx = (zAlpha[ia] == z0Var ? wzVar.idx : zAlpha[ia].idx)
                y[idx] = -Rprod.get(ia)
            for i in range(0, msize):
                if (wVars[i].isBasic):
                    int row = wVars[i].idx
                    getMaaRow (MbaRow, row)
                    y[row] = Mbuf[row*msize+col] - Rprod.dot(MbaRow)
        else: # var.isW()
            VectorNd Rprod = avec0
            VectorNd MbaRow = avec1

            Raa.getColumn (var.col, Rprod)
            for ia in range(0, asize):
                int idx = (zAlpha[ia] == z0Var ? wzVar.idx : zAlpha[ia].idx)
                y[idx] = Rprod.get(ia)

            for i in range(0, msize):
                if (wVars[i].isBasic):
                    int row = wVars[i].idx
                    getMaaRow (MbaRow, row)
                    y[row] = Rprod.dot(MbaRow)


    # @return void
    # @param y double[]
    def computeQv(y):
        if (!incrementalPivoting):
            if (y != qv):
                for i in range(0, msize):
                    y[i] = qv[i] 
        else:
            VectorNd qa = avec0
            VectorNd Rprod = avec1
            VectorNd MbaRow = avec2

            for ia in range(0, asize):
                qa.set (ia, qbuf[wAlpha[ia].idx])
            Raa.mul (Rprod, qa)
            for ia in range(0, asize):
                int idx = (zAlpha[ia] == z0Var ? wzVar.idx : zAlpha[ia].idx)
                y[idx] = -Rprod.get(ia)
            for i in range(0, msize):
                if (wVars[i].isBasic):
                    int row = wVars[i].idx
                    getMaaRow (MbaRow, row)
                    y[row] = qbuf[row] - Rprod.dot(MbaRow)

    # @return Variable
    # @param idx int
    def getBasic(idx):
        if (incrementalPivoting):
            if (zVars[idx].isBasic):
                return zVars[idx]
            else:
                return wVars[idx]
        else:
            return basicVars[idx]; 

    def solve(w, z, M, q, nr, eps):
        return solve (w, z, M, q, nr, eps, null)

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
    def solve(w, z, M, q, nr, eps, preset):
        Variable driveVar
        Variable blockingVar
        int numCand
        int origNumCand = 0
        boolean basisWasPreset = false

        int nc = nr+1
        boolean cycleCheck = False

        # print ("problem size = " + nr)
        if cycleCheckingEnabled:
            if (nr <= maxCycleCheckSize):
                cycleCheck = true
           
            else:
                print ("Warning: no cycle check because problem size exceeds " + maxCycleCheckSize)

        checkArgs(w, z, M, q, nr, eps)

        if (fullSolve or preset) != None : 
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

        double qmin = Double.POSITIVE_INFINITY
        int pivotCnt = 0
        int maxPivotCnt = 1000 # nr*nr

        Mbuf = M
        qbuf = q
        msize = nr
        
        if !incrementalPivoting :
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
            asize = 0
            Raa.setSize (asize, asize)
            avec0.setSize (asize)
            avec1.setSize (asize)
            avec2.setSize (asize)
            avec3.setSize (asize)

        for i in range(0, nr):
            Variable wvar = wVars[i]
            Variable zvar = zVars[i]

            wvar.init(W_VAR, zvar)
            zvar.init(Z_VAR, wvar)

            basicVars[i] = wvar
            nonBasicVars[i] = zvar

            qv[i] = q[i]

        z0Var = zVars[nr]
        z0Var.init (Z0, null)

        nonBasicVars[nr] = z0Var
        nonCompIdx = nr


        if (preset != null):

            int numActive = 0
            # initialize cover vector
            for i in range(0, nr):
                cvec[i] = 0
          
            for i in range(0, min(preset.length, nr)):
                if preset[i]:
                    zVars[i].isBasic = true
                    wVars[i].isBasic = false

                    basicVars[i] = zVars[i]
                    nonBasicVars[i] = wVars[i]
                    numActive++
                    
            for j in range(0, nr):
                if (zVars[j].isBasic):
                    for i in range(0, nr):
                        cvec[i] -= M[i*nr+j]
                else:
                    cvec[j] += 3

            if (numActive > 0):
             
                if (!computeMvFromBasis (1000*DOUBLE_PREC)):
                    print ("pre-basis ill-conditioned; ignoring ...")
                    for i in range(0, nr):
                        zVars[i].isBasic = false
                        wVars[i].isBasic = true
                        cvec[i] = 1

                        basicVars[i] = wVars[i]
                        nonBasicVars[i] = zVars[i]
                else:
                    basisWasPreset = true
            
        else:
            for i in range(0, nr):
                cvec[i] = 1
        
        if ((debug & SHOW_MINRATIO) != 0):
            print ("Initial qv:")
            NumberFormat fmt = new NumberFormat("%14.8f"):
            for i in range(0, nr):
                print (basicVars[i].getName() + " " +fmt.format(qv[i])):

        for i in range(0, nr):
            if (basisWasPreset):
                mv[i] = wVars[i].isBasic ? 3 : 1
            else:
                mv[i] = cvec[i]

        for i in range(0, nr):
            mv[i] = -mv[i]

        int imin = lexicoMinRatioTest (mv, qv, nr, -1, true)
        if (imin == -1):
            # then q >= 0 and so z = 0 is a solution
            computeSolution (w, z, M, q, qv, nr)
            # print ("basis: " + basisString(null))
            return SOLVED
        
        '''
        #     if (preset != null):
        #      {
         for i in range(0, nr):
        #         {
            if (zVars[i].isBasic):
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
        int r = imin
        int i_zx = r

        if ((debug & SHOW_MINRATIO) != 0):
            print ("Initial:")
        

        if (wVars[r].isBasic):
            driveVar = pivot (wVars[r], z0Var)
        else:
            driveVar = pivot (zVars[r], z0Var)

        if (cycleCheck):
            currentBasis.init (driveVar)

        # CandidateList listCopy = null

        while (pivotCnt < maxPivotCnt):
            # driving variable is indexed by r.
            # find the blocking variable from the r-th column
            computeMvCol (mv, driveVar)

            if ((debug & (SHOW_MINRATIO | SHOW_BASIS)) != 0):
                print ("Basis: " + basisString(driveVar))
            
            if ((debug & SHOW_MINRATIO) != 0):
                print ("epsilon=" + epsilon)
                print ("driving variable="+ driveVar.getName())
            
            int z_i = incrementalPivoting ? wzVar.idx : i_zx
            int s = lexicoMinRatioTest (mv, qv, nr, z_i, /*initial=*/false)
            if (s == -1):
                # then driving variable is unblocked, so we can't find a solution
                print ("unbounded ray")
                return UNBOUNDED_RAY
            
            elif (s == z_i):
                # zx blocks the driving variable
                pivot (z0Var, driveVar)
                if ((debug & SHOW_MINRATIO) != 0):
                    printQv ("Final:", qv, nr)
                computeSolution (w, z, M, q, qv, nr)
                # print ("basis: " + basisString(null))
                if (preset != null):
                    for i in range(0, min(preset.length, nr)):
                        preset[i] = zVars[i].isBasic; 
                return SOLVED
            else:
                driveVar = pivot (getBasic(s), driveVar)
            pivotCnt++

        return CYCLING_DETECTED

    # @return double
    def getEpsilon():
        return epsilon

    # @return boolean
    # @param j int 
    def wIsBasic(j):
        return wVars[j].isBasic 

    # @return void
    def getBasisColumn(iv, j):
        computeMvCol (iv, wVars[j])

    # @return Variable[]
    def getBasicVars():
        if (incrementalPivoting):
            Variable[] vars = new Variable[msize]

            for i in range(0, msize):
                if (wVars[i].isBasic):
                    vars[i] = wVars[i]                
                elif (zVars[i].isBasic):
                    vars[i] = zVars[i]
                else:
                    vars[i] = z0Var
            return vars
        else:
            return basicVars

    # @return Variable
    def getWzVar():
        return wzVar

    # @return String
    def getBasisString():
        return basisString(null)
