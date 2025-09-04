Notes
===============

Lecture Notes
-------------

Please check my general `lecture notes
<https://www.user.tu-berlin.de/mtoussai/teaching/#lecture-notes>`_.
Some of these are particularly relevant for understanding the code:

* The `Robot Kinematics and Dynamics Essentials <https://www.user.tu-berlin.de/mtoussai/notes/robotKin.html>`_ explains robot systems as multibody configurations, kinematics, Jacobians, inverse kinematics, dynamics, the default *Waypoint + Reference Motion + Control* control stack.

* The `Quaternion Lecture Note <https://www.user.tu-berlin.de/mtoussai/notes/quaternions.html>`_ explains quaternions properly (connecting to the Lie group notions, esp. the *log* and *exp* maps), which is an important representation of rotations
  
* The `Splines Lecture Note <https://www.user.tu-berlin.de/mtoussai/notes/splines.html>`_, which explains splines, in particular B-splines and the maths underlying the interpolation that is also happening within the BotOp control interface.

Code Notes
-------------

NLP interface
~~~~~~~~~~~~~

A general non-linear mathematical program (NLP) is of the form

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ f(x) ~~~\text{s.t.}~~~ g(x)\le 0,~ h(x) = 0  ~,\end{aligned}

with :math:`x\in{\mathbb{R}}^n`,
:math:`f:~ {\mathbb{R}}^n \to {\mathbb{R}}`,
:math:`g:~ {\mathbb{R}}^n \to {\mathbb{R}}^{d_g}`,
:math:`h:~ {\mathbb{R}}^n \to {\mathbb{R}}^{d_h}`,
:math:`b_l,b_u\in{\mathbb{R}}^n`. However, we want to explicitly also allow
for **least squares** costs (sum-of-squares), so that we extend the form
to

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ f(x) + r(x)^{\!\top\!}r(x) ~~~\text{s.t.}~~~ g(x)\le 0,~ h(x) = 0  ~,\end{aligned}

with :math:`r:~ {\mathbb{R}}^n \to {\mathbb{R}}^{d_r}`. In technical
terms, the solver needs to be provided with:

-  the problem *signature*: dimension :math:`n`, dimensions
   :math:`d_r, d_g, d_h`, bounds :math:`b_l, b_u \in {\mathbb{R}}^n`,

-  functions :math:`f, r, g, h`,   Jacobians for all,   Hessian for
   :math:`f`,

-  typically also an initialization sampler :math:`x_0 \sim p(x)`, that
   provides starting :math:`x_0`.

However, instead of providing a solver with separate functions
:math:`f, r, g, h`, we instead provide only a single differentiable
**feature** function :math:`\phi: X \to {\mathbb{R}}^K`, which stacks
all :math:`f,r,g,h` components to a single vector,

.. math::

   \begin{aligned}
   \phi(x) =  \left(\begin{array}{c}f_1(x) \\ r_1(x) \\ h_1(x) \\ g_1(x) \\ h_2(x) \\ \vdots\end{array}\right) 
   ~,\quad
   \rho =  \left(\begin{array}{c}\texttt{f}\\ \texttt{sos}\\ \texttt{eq}\\ \texttt{ineq}\\ \texttt{eq}\\ \vdots\end{array}\right)  ~,\end{aligned}

where the indicator vector :math:`\rho` informs the solver which
components of :math:`\phi` have to be treated as linear cost (``f``),
sum-of-squares cost (``sos``), equality constraint (``eq``), or
inequality constraint (``ineq``). (The order of stacking does not
matter.) In this convention, the NLP reads

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ {{\bf 1}}^{\!\top\!}\phi_\texttt{f}(x) + \phi_\texttt{sos}(x)^{\!\top\!}\phi_\texttt{sos}(x)
     ~~\text{s.t.}~~\phi_\texttt{ineq}(x) \le 0,~ \phi_\texttt{eq}(x) = 0 ~,\end{aligned}

where :math:`\phi_\texttt{sos}` is the subsets of ``sos``-features, etc.
Based on these conventions, the solver needs to be provided with:

-  the problem *signature*: dimension :math:`n`, feature types
   :math:`\rho`, bounds :math:`b_l, b_u \in {\mathbb{R}}^n`,

-  a single differentiable feature function
   :math:`\phi: X \to {\mathbb{R}}^K`, with Jacobian function
   :math:`J = \partial_x \phi(x)`,

-  optionally a Hessian function for the sum of all ``f``-terms,

-  and typically also an initialization sampler :math:`x_0 \sim p(x)`,
   that provides starting :math:`x_0`.

In the rai code, an abstract ``NLP`` is therefore declared as

::

     //signature
     uint dimension;  ObjectiveTypeA featureTypes;  arr bounds;

     //essential method
     virtual void evaluate(arr& phi, arr& J, const arr& x);

     //optional
     virtual arr  getInitializationSample(const arr& previousOptima={});
     virtual void getFHessian(arr& H, const arr& x);

The ``bounds`` is a :math:`2\times n` array, with the two rows being the lower and upper bound vector.
     
Yaml-Graph Files
~~~~~~~~~~~~~~~~

We use yaml-style files throughout. These are the file representation of
internal data structures such as dictionaries (anytype key-value maps)
used for parameter files or other structure data, but also for graphs.
The (semantic) extensions relative to yaml are:

-  An ``Include`` node allows to hierarchically include files. This means
   that while each local file can be parsed with a standard yaml parser,
   an outer loop has to check for ``Include`` nodes and coordinate loading
   sub-files.

-  As an implication of the above, we allow for a special *path* type,
   as URLs embraced by ``<...>``. This becomes necessary as file values
   need to be interpreted relative to the path of the loading file. So
   when such a file is parsed we not only store the filename string, but
   also the path of the loading file to ensure we know its absolute
   path.

-  We also allow ``Edit`` and ``Delete`` nodes, which allow us to
   edit/overwrite the value of previously defined nodes, as well as
   delete previously defined nodes.

-  Finally, the name of a node can include a list of parents: E.g. ``A (B
   C): { shape: box }`` denotes a node with key ``A`` that is a child of ``B``
   and ``C``. The semantics of this is that ``A`` is a (directed) edge
   between B and C. This is analogous to a dot declaration ``B -> C [
   shape=box ]``.

-  Note that all of the above is still yaml syntax, the outer parser
   only adds additional interpretation (post-processing) of ``Include,
   Edit, Delete`` tags, ``<..>`` values, and ``(..)`` in names.

Within the code, .g-files are used to represent parameter files, robotic
configurations (:math:`\sim` URDF), 1st order logic, factor graphs,
optimization problems. The underlying data structure is used, e.g., as
any-type container, Graph, or auto-conversion to python dictionaries.

Subgraphs may contain nodes that have parents from the containing graph,
or from other subgraphs of the containing graph. Some methods of the
``Graph`` class (to find nodes by key or type) allow to specify whether
also nodes in subgraphs or parentgraphs are to be searched. This
connectivity across (sub)-graphs e.g. allows to represent logic
knowledge bases.

The `Editing Configurations tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_3_import_edit.html>`_
shows how this file syntax is used to specify robot and environment
configurations.

