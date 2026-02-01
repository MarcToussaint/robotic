import numpy as np

class OT():
    f = 0
    sos = 1
    ineq = 2
    eq = 3

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
    phi_sos is a vector of least cost terms
    phi_eq is a vector of equality constraints
    phi_ineq is a vector of inequality constraints
    B[0] and B[1] in R^n are, respectively, the lower and upper box bounds
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
        triplet format; the C++ solvers exploit such sparse Jacobians,
        e.g., to efficiently compute Gauss-Newton steps.

        Parameters
        ------
        x: np.array, 1-D

        Returns
        ------
        phi: np.array 1-D
        J: np.array 2-D.  J[i,j] is derivative of feature i w.r.t variable j, potentially sparse
        """
        raise NotImplementedError()

    def getDimension(self):
        """
        Returns
        -----
        output: dimensionality of x
        """
        raise NotImplementedError()

    def getFeatureTypes(self):
        """
        Returns
        -----
        output: list of feature Types
        """
        return NotImplementedError()

    def getBounds(self):
        """
        returns 2-times-n array, where n is the dimensionality of x
        B[0] is the lower and B[1] the upper bounds, so that B[0] <= x <= B[1]

        Returns
        ------
        B: np.array 2D (2-times-n)
        """
        n = self.getDimension()
        return np.tile([[-1.], [+1.]], (1,n))

    def getFHessian(self, x):
        """
        Optionally returns a Hessian of the sum of all phi_f objectives
        The Hessian is by default approximated Gauss-Newton w.r.t. the phi_sos objectives; this
        method allows to return an explicit Hessian also for phi_f objectives that is added to the
        Gauss-Newton of phi_sos objectives. Needs to be sparse if $J$ is sparse. Default: empty array (zero).

        Returns
        -----
        Hessian: np.array 2D, potentially sparse
        """
        return []

    def getInitializationSample(self):
        """
        returns a sample to initialize an optimization, not necessarily feasible. Default: uniform within bounds

        Returns
        -----
        x:  np.array 1-D
        """
        bounds = self.getBounds()
        return np.random.uniform(low=bounds[0,:], high=bounds[1,:])

    def report(self, verbose):
        """
        displays semantic information on the last query. Default: empty string

        Returns
        ----
        output: string
        """
        return ''
