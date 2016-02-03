class LemkeSolverBase:
    static final int SOLVED = 1;
    static final int UNBOUNDED_RAY = 2;
    static final int CYCLING_DETECTED = 3;

    static final int BASIC = 1;
    static final int NEW = 3;

    protected static double DOUBLE_PREC = 2.2204460492503131e-16;
    protected static double WORKING_PREC = 100*DOUBLE_PREC;
    protected double epsilon; // XXX determine this

    static final double AUTO_EPSILON = -1.0;

    protected int minRatioMethod = BASIC;

    static final int SHOW_BASIS = 0x01;
    static final int SHOW_MINRATIO = 0x02;
    static final int SHOW_LEXICO_MINRATIO = 0x06;

    protected int debug = 0;

    static final int Z_VAR = 0x8000;
    static final int W_VAR = 0x0000;
    static final int Z0 =    Z_VAR | 0x0;

    protected static class Variable
    {
        String name;
        int type;
        boolean isBasic;
        Variable complement;
        int col;
        int idx;

        boolean isW()
        {
            return (type & Z_VAR) == 0;
        }

        boolean isZ()
        {
            return (type & Z_VAR) != 0;
        }

        boolean isZ0()
        {
            return complement == null;
        }

        void setIndex (int i)
        {
            idx = i;
        }

        void init (int type, Variable complement)
        {
            if ((type & Z_VAR) != 0)
            {
                isBasic = false;
            }
            else
            {
                isBasic = true;
            }
            col = idx;
            this.complement = complement;
            this.type = type;
        }

        void set (Variable var)
        {
            name = var.name;
            type = var.type;
            isBasic = var.isBasic;
            complement = var.complement;
            col = var.col;
        }

        String getName()
        {
            if (name != null)
            {
                return name;
            }
            else if (isZ())
            {
                if (complement == null)
                {
                    return "z0";
                }
                else
                {
                    return "z" + (idx+1);
                }
            }
            else
            {
                return "w" + (idx+1); 
            }
        }
    }

    protected double[] qvNew = new double[0];
    protected boolean[] isCandidate = new boolean[0];
    protected int[] candidates = new int[0];
    protected double[] iv = new double[0];

    protected Object growObjectArray (
            Object oldArray, int length, Class classType)
    {

        int oldLength = Array.getLength (oldArray);
        Object newArray = Array.newInstance (classType, length);

        for i in range(0, oldLength)
        {
            Array.set (newArray, i, Array.get (oldArray, i));
        }
        for i in range(oldLength, length)
        {
            try
            {
                Array.set (newArray, i, classType.newInstance());
            }
            catch (Exception e)
            {
                e.printStackTrace();
                System.exit(1); 
            }
        }
        return newArray;
    }   

    protected int cumulativePivotCnt = 0;


    protected void allocateSpace (int num)
    {

        if (qvNew.length < num)
        {
            qvNew = new double[num]; 
            isCandidate = new boolean[num];
            candidates = new int[num];
            iv = new double[num];
        }
    }

    #
    # Items relating to cycle checking ...
    #

    protected boolean cycleCheckingEnabled = false;
    protected int maxCycleCheckSize = 64;
    protected int maxBasisHistory = 512;

    boolean cycleCheckingEnabled()
    {
        return cycleCheckingEnabled;
    }

    void setCycleChecking (boolean enable)
    {
        cycleCheckingEnabled = enable;
    }

    int getPivotCount()
    {
        return cumulativePivotCnt;
    }

    void resetPivotCount ()
    {
        cumulativePivotCnt = 0;
    }

    void setPivotCount (int cnt)
    {
        cumulativePivotCnt = cnt;
    }

    void setDebug (int code)
    {
        debug = code;
    }

    private int makeZeroList (int[] candidates, double[] mv, double[] qv, int nr, double maxQ)
    {
        double tol0 = Math.max(epsilon, maxQ*WORKING_PREC);
        int numCand = 0;
        System.out.println ("adding zeros");
        for k in range(0, nr)
        {
            if (qv[k] < tol0)
            {
                candidates[numCand++] = k;
                System.out.println ("k=" + k + " qv=" + qv[k]);
            }
        }
        System.out.println (" num candidates=" + numCand++);
        return numCand;
    }

    private int minRatio (
            int[] candidates, int numc, double[] mv, double[] qv, int z_i,
            boolean tie, boolean takeFirst)

    {
        double minRatio = Double.POSITIVE_INFINITY;
        int imin = -1;

        for k in range(0, numc)
        {
            int i = candidates[k];
            double ratio = -qv[i]/mv[i];
            if (qv[i] > 0 || tie)
            {
                if (ratio < minRatio)
                {
                    minRatio = ratio;
                    imin = i;
                }
            }
            else
            {
                minRatio = 0;
                imin = i;
            }
        }

        if (imin != -1 && takeFirst)
        {
            candidates[0] = imin;
            return 1;
        }

        int newc = 0;
        for k in range(0, numc)
        {
            int i = candidates[k];
            double ratio = -qv[i]/mv[i];
            if (Math.abs(ratio-minRatio) <= epsilon || (!tie && qv[i] <= 0))
            {
                candidates[newc++] = i;
                if (i == z_i)
                {
                    candidates[0] = i;
                    return 1;
                }
            }
        }
        return newc;
    }

    protected abstract boolean wIsBasic (int j);

    protected abstract void getBasisColumn (double[] iv, int j);

    protected abstract Variable[] getBasicVars ();

    protected abstract Variable getWzVar();

    private boolean[] getDisplayedRows (
            int[] candidates, int numc, int numv)
    {

        boolean[] displayRow = new boolean[numv];
        for i in range(0, numc)
        {
            displayRow[candidates[i]] = true; 
        }
        return displayRow;
    }

    private int initCandidates (double[] mv, double[] qv, int numv,
            boolean initial)
    {
        int numc = 0;
        if (initial)
        {
            for i in range(0, numv)
            {
                if (qv[i] < -epsilon)
                {
                    candidates[numc++] = i;
                }
            }
        }
        else
        {
            for i in range(0, numv)
            {
                if (mv[i] < -epsilon)
                {
                    candidates[numc++] = i;
                }
            } 
        }
        return numc;
    }

    int lexicoMinRatioTest (
            double[] mv, double[] qv, int numv, int z_i, boolean initial)
    {

        int blocking_i = -1;
        boolean[] displayRow = null; // rows to print for debug

        boolean printCols = (debug & SHOW_MINRATIO) != 0;
        boolean printLexicoCols =
            (debug & SHOW_LEXICO_MINRATIO) == SHOW_LEXICO_MINRATIO;

        int numc = initCandidates (mv, qv, numv, initial);
        if (numc == 0)
        {
            if (printCols)
            {
                printMinRatioInfo (mv, qv, numv, z_i, -1, null, 0);
            }
            return -1;
        }
        numc = minRatio (candidates, numc, mv, qv, z_i, initial, initial);

        if (numc > 1)
        {

            if (printLexicoCols)
            {
                printMinRatioInfo (mv, qv, numv, z_i, -1, candidates, numc);
                displayRow = getDisplayedRows (candidates, numc, numv);
            }
            for j in range(0, numv && numc>1)
            {
                if (wIsBasic (j))
                {
                    // then iv is simply e(iv_j)
                    numc = minRatioElementaryTest (candidates, j, numc);
                    if ((debug & SHOW_LEXICO_MINRATIO) != 0)
                    {
                        for i in range(0, numv)
                        {
                            iv[i] = (i==j ? 1 : 0); 
                        }
                    }
                }
                else
                {
                    getBasisColumn (iv, j);
                    for i in range(0, numv)
                    {
                        iv[i] = -iv[i]; 
                    }
                    numc = minRatio (candidates, numc, mv, iv, -1, true, false);
                }
                if (printLexicoCols)
                {
                    int b_i = (numc==1 || j==numv-1) ? candidates[0] : -1;
                    System.out.println ("TIE break, basis column " + j + " :");
                    printMinRatioInfo (mv, iv, numv, z_i, b_i,
                            candidates, numc, displayRow); 
                    displayRow = getDisplayedRows (candidates, numc, numv);
                }
            }
            if (numc > 1)
            {
                System.out.println (
                        "Warning: lexicoMinRatio finished with " + numc + " candidates");
            }
        }
        blocking_i = candidates[0];

        if (printCols && displayRow==null)
        {
            numc = initCandidates (mv, qv, numv, initial);
            numc = minRatio (candidates, numc, mv, qv, z_i, initial, initial);
            printMinRatioInfo (mv, qv, numv, z_i,
                    blocking_i, candidates, numc);
        }
        return blocking_i;
    }

    LemkeSolverBase()
    {

    }

    /**
     * Performs a minimun ratio test on an elementary vector e(ei);
     * i.e., e(i) = 0 if i != ei and e(ei) = 1.
     *
     * All we do in this case is eliminate any node from
     * the list whose index is ei.   
     */
    protected int minRatioElementaryTest (int[] candidates, int ei,
            int numCand)
    {
        int k;
        for (k=0; k<numCand; k++)
        {
            if (candidates[k] == ei)
            {
                break;
            }
        }
        if (k == numCand)
        {
            return numCand;
        }
        else
        {
            while (k < numCand-1)
            {
                candidates[k] = candidates[k+1];
                k++;
            }
            return numCand-1;
        }
    }

    private boolean isCandidate (int i, int[] candidates, int numCand)
    {
        for k in range(0, numCand)
        {
            if (candidates[k] == i)
            {
                return true; 
            }
        }
        return false;
    }

    protected void printBasis (Variable driveVar)
    {
        System.out.println ("Basis: " + basisString(driveVar));
    }


    protected String basisString (Variable driveVar)
    {
        return "";
    }

    protected void printMinRatioInfo (
            double[] mv, double[] qv, int numv, 
            int z_i, int blocking_i, int[] candidates, int numc)
    {

        printMinRatioInfo (mv, qv, numv, z_i, blocking_i,
                candidates, numc, null);
    }

    private int[] getDisplayIndices (
            Variable[] basicVars, int numv, boolean[] displayRow, int z_i)
    {
        int[] idxs;

        if (displayRow == null)
        {
            idxs = new int[numv];
            for i in range(0, numv)
            {
                idxs[i] = i; 
            }
        }
        else
        {
            int numdisp = 0;
            for i in range(0, numv)
            {
                if (displayRow[i])
                {
                    numdisp++; 
                }
            }
            idxs = new int[numdisp];
            int k = 0;
            for i in range(0, numv)
            {
                if (displayRow[i])
                {
                    idxs[k++] = i;
                }
            }
        }
        
        '''
        //     // bubble sort indices by variable index
        //     for i in range(0, idxs.length-1)
        //      {
        //for j in range(i+1, idxs.length)
            //         {
        //    int idx_i = basicVars[idxs[i]].idx;
        //       int idx_j = basicVars[idxs[j]].idx;
        //       if (basicVars[idxs[i]].isZ())
        //        {
        //idx_i = getWzVar().idx;
        //        }
        //       if (basicVars[idxs[j]].isZ())
        //        {
        //idx_j = getWzVar().idx;
        //        }
        //       if (idx_i > idx_j)
        //        {
        //int k = idxs[i]; idxs[i] = idxs[j]; idxs[j] = k;
        //        }
        //         }
        //      }
        '''

        return idxs;
    }

    private int getMaxNameLength (Variable[] vars, int numv)
    {
        int maxNameLength = 0;
        for i in range(0, numv)
        {
            if (vars[i].getName().length() > maxNameLength)
            {
                maxNameLength = vars[i].getName().length();
            }
        }
        return maxNameLength;
    }

    protected void printQv (String msg, double[] qv, int numv)
    {
        Variable[] basicVars = getBasicVars();
        int maxNameLength = getMaxNameLength (basicVars, numv);

        NumberFormat ffmt = new NumberFormat ("%24g");
        StringBuffer sbuf = new StringBuffer(256);

        if (msg != null)
        {
            System.out.println (msg); 
        }
        for i in range(0, numv)
        {

            sbuf.setLength(0);
            sbuf.append ("  ");
            sbuf.append (basicVars[i].getName());
            while (sbuf.length() < maxNameLength+2)
            {
                sbuf.insert (2, ' '); 
            }         
            sbuf.append ("   ");
            sbuf.append (ffmt.format(qv[i]));
            System.out.println (sbuf.toString());
        }
    }

    protected void printMinRatioInfo (
            double[] mv, double[] qv, int numv, 
            int z_i, int blocking_i, int[] candidates, int numc,
            boolean[] displayRow)
    {
        Variable[] basicVars = getBasicVars();

        int idxs[] = getDisplayIndices (basicVars, numv, displayRow, z_i);
        boolean[] isCandidate = new boolean[numv];

        if (candidates != null)
        {
            for i in range(0, numc)
            {
                isCandidate[candidates[i]] = true; 
            }
        }

        NumberFormat ifmt = new NumberFormat ("%3d");
        NumberFormat ffmt = new NumberFormat ("%24g");
        StringBuffer sbuf = new StringBuffer(256);

        if (blocking_i != -1)
        {
            System.out.println (
                    "blocking variable=" + basicVars[blocking_i].getName());
        }

        int maxNameLength = getMaxNameLength (basicVars, numv);

        for k in range(0, idxs.length)
        {
            int i = idxs[k];
            sbuf.setLength(0);
            sbuf.append (basicVars[i].isZ() ? "z " : "  ");
            sbuf.append (basicVars[i].getName());
            while (sbuf.length() < maxNameLength+2)
            {
                sbuf.insert (2, ' '); 
            }
            sbuf.append (' ');

            sbuf.append (isCandidate[i] ? '*' : ' ');
            sbuf.append (i == blocking_i ? "XX " : "   ");
            sbuf.append (ffmt.format(mv[i]));
            sbuf.append (ffmt.format(qv[i]));
            sbuf.append (ffmt.format(-qv[i]/mv[i]));
            System.out.println (sbuf.toString());
        }
    }

    double getEpsilon ()
    {
        return epsilon; 
    }

