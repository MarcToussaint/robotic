import numpy as np

class OT():
    none = 0
    f = 1
    sos = 2
    ineq = 3
    eq = 4


class NLP():
    """
    Non-Linear Mathematical Program

    min_x  1^T phi_f(x) + phi_sos(x)^T phi_sos(x)
    s.t.     phi_eq(x) = 0
             phi_ineq(x) <= 0
             B[0] <= x <= B[1]

    where:
    x is a continous variable, in vector space R^n
    phi_f is a vector of cost terms
    phi_sos is a vector of sum-of-square cost terms
    phi_eq is a vector of equality constraints
    phi_ineq is a vector of inequality constraints
    B[0] and B[1] in R^n are, respectively, the lower and upper box bounds


    See Also:
    -----
    NLPTraced

    """

    def __init__(self, *args, **kwargs):
        pass

    def evaluate(self, x):
        """
        query the NLP at a point x; returns the tuple (phi,J), which is
        the feature vector and its Jacobian; features define cost terms,
        sum-of-square (sos) terms, inequalities, and equalities depending
        on 'getFeatureTypes'.

        If the returned Jacobian has 3 columns and #rows non-equal to
        dim(phi) it should be interpreted by solvers as sparse in
        triple format; the C++ solvers exploit such sparse Jacobians,
        e.g., to efficiently compute Gauss-Newton steps.

        Parameters
        ------
        x: np.array, 1-D

        Returns
        ------
        phi: np.array 1-D
        J: np.array 2-D.  J[i,j] is derivative of feature i w.r.t variable j

        """
        raise NotImplementedError()

    def getDimension(self):
        """
        return the dimensionality of x

        Returns
        -----
        output: integer

        """
        raise NotImplementedError()

    def getBounds(self):
        """
        returns 2-times-n array, where n is the dimensionality of x
        B[0] is the lower and B[1] the upper bounds, so that B[0] <= x <= B[1]

        Returns
        ------
        B: np.array 2D (2-times-n)

        """
        n = self.getDimension()
        return np.tile([[-np.Inf], [+np.Inf]], (1,n))

    def getFeatureTypes(self):
        """
        returns
        -----
        output: list of feature Types

        """
        return [OT.f]

    def getFHessian(self, x):
        """
        Returns Hessian of the $f$ objective

        Default: all zeros

        Returns
        -----
        hessian: np.array 2D

        """
        n = self.getDimension()
        return np.zeros((n, n))

    def getInitializationSample(self):
        """
        returns a sample (e.g. uniform within bounds) to initialize an optimization -- not necessarily feasible

        Returns
        -----
        x:  np.array 1-D

        """
        return np.ones(self.getDimension())

    def report(self, verbose):
        """
        displays semantic information on the last query

        Parameters
        ----

        Returns
        ----
        output: string
        """
        raise NotImplementedError()
