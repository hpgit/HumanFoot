class LemkeSolverBase:
    '''
    static final int SOLVED = 1
    static final int UNBOUNDED_RAY = 2
    static final int CYCLING_DETECTED = 3

    static final int BASIC = 1
    static final int NEW = 3

    protected static double DOUBLE_PREC = 2.2204460492503131e-16
    protected static double WORKING_PREC = 100*DOUBLE_PREC
    protected double epsilon; // XXX determine this

    static final double AUTO_EPSILON = -1.0

    protected int minRatioMethod = BASIC

    static final int SHOW_BASIS = 0x01
    static final int SHOW_MINRATIO = 0x02
    static final int SHOW_LEXICO_MINRATIO = 0x06

    protected int debug = 0

    static final int Z_VAR = 0x8000
    static final int W_VAR = 0x0000
    static final int Z0 =    Z_VAR | 0x0
    '''

    def __init__(self):
        self.SOLVED = 1
        self.UNBOUNDED_RAY = 2
        self.CYCLING_DETECTED = 3

        self.BASIC = 1
        self.NEW = 3

        self.DOUBLE_PREC = 2.2204460492503131e-16
        self.WORKING_PREC = 100*self.DOUBLE_PREC
        self.epsilon = -1.0  # XXX determine this

        self.AUTO_EPSILON = -1.0

        self.minRatioMethod = self.BASIC

        self.SHOW_BASIS = 0x01
        self.SHOW_MINRATIO = 0x02
        self.SHOW_LEXICO_MINRATIO = 0x06

        self.debug = 0

        self.Z_VAR = 0x8000
        self.W_VAR = 0x0000
        self.Z0 = self.Z_VAR | 0x0

        self.qvNew = [0.]*0
        self.isCandidate = [False]*0
        self.candidates = [0]*0
        self.iv = [0.]*0

        self.cumulativePivotCnt = 0

        #
        # Items relating to cycle checking ...
        #

        self.cycleCheckingEnabled = False
        self.maxCycleCheckSize = 64
        self.maxBasisHistory = 512

    class Variable:
        def __init__(self):
            self.name = None
            self.varType = None
            self.isBasic = None
            self.complement = None
            self.col = None
            self.idx = None

        # @return boolean
        def isW(self):
            return (self.varType & self.Z_VAR) == 0

        # @return boolean
        def isZ(self):
            return (self.varType & self.Z_VAR) != 0

        # @return boolean
        def isZ0(self):
            return self.complement is None

        # @return boolean
        # @param i int
        def setIndex(self, i_):
            self.idx = i_

        # @return void
        # @param type int
        # @param complement Variable
        def init(self, varType_, complement):
            if (varType_ & self.Z_VAR) != 0:
                self.isBasic = False
            else:
                self.isBasic = True
            self.col = self.idx
            self.complement = complement
            self.varType = varType_

        # @return void
        # @param var Variable
        def set(self, var_):
            self.name = var_.name
            self.varType = var_.varType
            self.isBasic = var_.isBasic
            self.complement = var_.complement
            self.col = var_.col

        # @return string
        def getName(self):
            if self.name is not None:
                return self.name
            elif self.isZ():
                if self.complement is None:
                    return "z0"
                else:
                    return "z" + str(self.idx+1)
            else:
                return "w" + str(self.idx+1)

    # @return Object
    # @param oldArray Object
    # @param length int
    # @param classType Class
    def growObjectArray (Object oldArray, int length, Class classType):
    {

        int oldLength = Array.getLength (oldArray)
        Object newArray = Array.newInstance (classType, length)

        for i in range(0, oldLength)
        {
            Array.set (newArray, i, Array.get (oldArray, i))
        }
        for i in range(oldLength, length)
        {
            try
            {
                Array.set (newArray, i, classType.newInstance())
            }
            catch (Exception e)
            {
                e.printStackTrace()
                System.exit(1)
            }
        }
        return newArray
    }   



    # @return void
    # @param num int
    def allocateSpace(self, num):
        if qvNew.length < num:
            qvNew = new double[num]
            isCandidate = new boolean[num]
            candidates = new int[num]
            iv = new double[num]

    #
    # Items relating to cycle checking ...
    #

    # @return boolean
    def cycleCheckingEnabled():
    {
        return cycleCheckingEnabled
    }

    # @return void
    # @param enable boolean
    def setCycleChecking (boolean enable):
        cycleCheckingEnabled = enable
    
    # @return int
    def getPivotCount():
        return cumulativePivotCnt

    # @return void
    def resetPivotCount ():
    {
        cumulativePivotCnt = 0
    }

    # @return void
    # @param cnt int
    def setPivotCount (int cnt):
    {
        cumulativePivotCnt = cnt
    }

    # @return void
    # @param code int
    void setDebug (int code)
    {
        debug = code
    }

    private int makeZeroList (int[] candidates, double[] mv, double[] qv, int nr, double maxQ)
    {
        double tol0 = Math.max(epsilon, maxQ*WORKING_PREC)
        int numCand = 0
        System.out.println ("adding zeros")
        for k in range(0, nr)
        {
            if qv[k] < tol0:
            {
                candidates[numCand++] = k
                System.out.println ("k=" + k + " qv=" + qv[k])
            }
        }
        System.out.println (" num candidates=" + numCand++)
        return numCand
    }

    private int minRatio (
            int[] candidates, int numc, double[] mv, double[] qv, int z_i,
            boolean tie, boolean takeFirst)

    {
        double minRatio = Double.POSITIVE_INFINITY
        int imin = -1

        for k in range(0, numc)
        {
            int i = candidates[k]
            double ratio = -qv[i]/mv[i]
            if qv[i] > 0 || tie:
            {
                if ratio < minRatio:
                {
                    minRatio = ratio
                    imin = i
                }
            }
            else:
            {
                minRatio = 0
                imin = i
            }
        }

        if imin != -1 && takeFirst:
        {
            candidates[0] = imin
            return 1
        }

        int newc = 0
        for k in range(0, numc)
        {
            int i = candidates[k]
            double ratio = -qv[i]/mv[i]
            if Math.abs(ratio-minRatio) <= epsilon || (!tie && qv[i] <= 0):
            {
                candidates[newc++] = i
                if i == z_i:
                {
                    candidates[0] = i
                    return 1
                }
            }
        }
        return newc
    }

    protected abstract boolean wIsBasic (int j)

    protected abstract void getBasisColumn (double[] iv, int j)

    protected abstract Variable[] getBasicVars ()

    protected abstract Variable getWzVar()

    private boolean[] getDisplayedRows (
            int[] candidates, int numc, int numv)
    {

        boolean[] displayRow = new boolean[numv]
        for i in range(0, numc)
        {
            displayRow[candidates[i]] = true; 
        }
        return displayRow
    }

    private int initCandidates (double[] mv, double[] qv, int numv,
            boolean initial)
    {
        int numc = 0
        if initial:
        {
            for i in range(0, numv)
            {
                if qv[i] < -epsilon:
                {
                    candidates[numc++] = i
                }
            }
        }
        else:
        {
            for i in range(0, numv)
            {
                if mv[i] < -epsilon:
                {
                    candidates[numc++] = i
                }
            } 
        }
        return numc
    }

    int lexicoMinRatioTest (
            double[] mv, double[] qv, int numv, int z_i, boolean initial)
    {

        int blocking_i = -1
        boolean[] displayRow = null; // rows to print for debug

        boolean printCols = (debug & SHOW_MINRATIO) != 0
        boolean printLexicoCols =
            (debug & SHOW_LEXICO_MINRATIO) == SHOW_LEXICO_MINRATIO

        int numc = initCandidates (mv, qv, numv, initial)
        if numc == 0:
        {
            if printCols:
            {
                printMinRatioInfo (mv, qv, numv, z_i, -1, null, 0)
            }
            return -1
        }
        numc = minRatio (candidates, numc, mv, qv, z_i, initial, initial)

        if numc > 1:
        {

            if printLexicoCols:
            {
                printMinRatioInfo (mv, qv, numv, z_i, -1, candidates, numc)
                displayRow = getDisplayedRows (candidates, numc, numv)
            }
            for j in range(0, numv && numc>1)
            {
                if wIsBasic (j):
                {
                    // then iv is simply e(iv_j)
                    numc = minRatioElementaryTest (candidates, j, numc)
                    if (debug & SHOW_LEXICO_MINRATIO) != 0:
                    {
                        for i in range(0, numv)
                        {
                            iv[i] = (i==j ? 1 : 0); 
                        }
                    }
                }
                else:
                {
                    getBasisColumn (iv, j)
                    for i in range(0, numv)
                    {
                        iv[i] = -iv[i]; 
                    }
                    numc = minRatio (candidates, numc, mv, iv, -1, true, false)
                }
                if printLexicoCols:
                {
                    int b_i = (numc==1 || j==numv-1) ? candidates[0] : -1
                    System.out.println ("TIE break, basis column " + j + " :")
                    printMinRatioInfo (mv, iv, numv, z_i, b_i,
                            candidates, numc, displayRow); 
                    displayRow = getDisplayedRows (candidates, numc, numv)
                }
            }
            if numc > 1:
            {
                System.out.println (
                        "Warning: lexicoMinRatio finished with " + numc + " candidates")
            }
        }
        blocking_i = candidates[0]

        if printCols && displayRow==null:
        {
            numc = initCandidates (mv, qv, numv, initial)
            numc = minRatio (candidates, numc, mv, qv, z_i, initial, initial)
            printMinRatioInfo (mv, qv, numv, z_i,
                    blocking_i, candidates, numc)
        }
        return blocking_i
    }

    '''
    /**
     * Performs a minimun ratio test on an elementary vector e(ei)
     * i.e., e(i) = 0 if i != ei and e(ei) = 1.
     *
     * All we do in this case is eliminate any node from
     * the list whose index is ei.   
     */
    '''
    protected int minRatioElementaryTest (int[] candidates, int ei, int numCand)
    {
        int k
        for (k=0; k<numCand; k++)
        {
            if candidates[k] == ei:
            {
                break
            }
        }
        if k == numCand:
        {
            return numCand
        }
        else:
        {
            while (k < numCand-1)
            {
                candidates[k] = candidates[k+1]
                k++
            }
            return numCand-1
        }
    }

    private boolean isCandidate (int i, int[] candidates, int numCand)
    {
        for k in range(0, numCand)
        {
            if candidates[k] == i:
            {
                return true; 
            }
        }
        return false
    }

    protected void printBasis (Variable driveVar)
    {
        System.out.println ("Basis: " + basisString(driveVar))
    }


    protected String basisString (Variable driveVar)
    {
        return ""
    }

    protected void printMinRatioInfo (
            double[] mv, double[] qv, int numv, 
            int z_i, int blocking_i, int[] candidates, int numc)
    {

        printMinRatioInfo (mv, qv, numv, z_i, blocking_i,
                candidates, numc, null)
    }

    private int[] getDisplayIndices (
            Variable[] basicVars, int numv, boolean[] displayRow, int z_i)
    {
        int[] idxs

        if displayRow == null:
        {
            idxs = new int[numv]
            for i in range(0, numv)
            {
                idxs[i] = i; 
            }
        }
        else:
        {
            int numdisp = 0
            for i in range(0, numv)
            {
                if displayRow[i]:
                {
                    numdisp++; 
                }
            }
            idxs = new int[numdisp]
            int k = 0
            for i in range(0, numv)
            {
                if displayRow[i]:
                {
                    idxs[k++] = i
                }
            }
        }
        
        '''
        //     // bubble sort indices by variable index
        //     for i in range(0, idxs.length-1)
        //      {
        //for j in range(i+1, idxs.length)
            //         {
        //    int idx_i = basicVars[idxs[i]].idx
        //       int idx_j = basicVars[idxs[j]].idx
        //       if basicVars[idxs[i]].isZ():
        //        {
        //idx_i = getWzVar().idx
        //        }
        //       if basicVars[idxs[j]].isZ():
        //        {
        //idx_j = getWzVar().idx
        //        }
        //       if idx_i > idx_j:
        //        {
        //int k = idxs[i]; idxs[i] = idxs[j]; idxs[j] = k
        //        }
        //         }
        //      }
        '''

        return idxs
    }

    private int getMaxNameLength (Variable[] vars, int numv)
    {
        int maxNameLength = 0
        for i in range(0, numv)
        {
            if vars[i].getName().length() > maxNameLength:
            {
                maxNameLength = vars[i].getName().length()
            }
        }
        return maxNameLength
    }

    protected void printQv (String msg, double[] qv, int numv)
    {
        Variable[] basicVars = getBasicVars()
        int maxNameLength = getMaxNameLength (basicVars, numv)

        NumberFormat ffmt = new NumberFormat ("%24g")
        StringBuffer sbuf = new StringBuffer(256)

        if msg != null:
        {
            System.out.println (msg); 
        }
        for i in range(0, numv)
        {

            sbuf.setLength(0)
            sbuf.append ("  ")
            sbuf.append (basicVars[i].getName())
            while (sbuf.length() < maxNameLength+2)
            {
                sbuf.insert (2, ' '); 
            }         
            sbuf.append ("   ")
            sbuf.append (ffmt.format(qv[i]))
            System.out.println (sbuf.toString())
        }
    }

    protected void printMinRatioInfo (
            double[] mv, double[] qv, int numv, 
            int z_i, int blocking_i, int[] candidates, int numc,
            boolean[] displayRow)
    {
        Variable[] basicVars = getBasicVars()

        int idxs[] = getDisplayIndices (basicVars, numv, displayRow, z_i)
        boolean[] isCandidate = new boolean[numv]

        if candidates != null:
        {
            for i in range(0, numc)
            {
                isCandidate[candidates[i]] = true; 
            }
        }

        NumberFormat ifmt = new NumberFormat ("%3d")
        NumberFormat ffmt = new NumberFormat ("%24g")
        StringBuffer sbuf = new StringBuffer(256)

        if blocking_i != -1:
        {
            System.out.println (
                    "blocking variable=" + basicVars[blocking_i].getName())
        }

        int maxNameLength = getMaxNameLength (basicVars, numv)

        for k in range(0, idxs.length)
        {
            int i = idxs[k]
            sbuf.setLength(0)
            sbuf.append (basicVars[i].isZ() ? "z " : "  ")
            sbuf.append (basicVars[i].getName())
            while (sbuf.length() < maxNameLength+2)
            {
                sbuf.insert (2, ' '); 
            }
            sbuf.append (' ')

            sbuf.append (isCandidate[i] ? '*' : ' ')
            sbuf.append (i == blocking_i ? "XX " : "   ")
            sbuf.append (ffmt.format(mv[i]))
            sbuf.append (ffmt.format(qv[i]))
            sbuf.append (ffmt.format(-qv[i]/mv[i]))
            System.out.println (sbuf.toString())
        }
    }

    # @return double
    def getEpsilon(self):
        return epsilon

