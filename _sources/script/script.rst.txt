Lecture Script
==============

Introduction
------------

Reference material
~~~~~~~~~~~~~~~~~~

In terms of background, please refer to the Maths for Intelligent
Systems
[<https://www.user.tu-berlin.de/mtoussai/teaching/Lecture-Maths.pdf>] as
well as the Intro to Robotics
[<https://www.user.tu-berlin.de/mtoussai/teaching/Lecture-Robotics.pdf>]
lecture scripts. Here a list of further teaching material:

-  Craig, J.J.: *Introduction to robotics: mechanics and control*.
   Addison-Wesley New York, 1989. (3rd edition 2006)

-  Steven M. LaValle: *Planning Algorithms*. Cambridge University Press,
   2006.

   **online:** http://planning.cs.uiuc.edu/

-  VideoLecture by Oussama Khatib:
   <http://videolectures.net/oussama_khatib/>

   (focus on kinematics, dynamics, control)

-  Oliver Brock’s lecture
   <http://www.robotics.tu-berlin.de/menue/teaching/>

-  Stefan Schaal’s lecture Introduction to Robotics:
   <http://www-clmc.usc.edu/Teaching/TeachingIntroductionToRoboticsSyllabus>

   (focus on control, useful: Basic Linear Control Theory (analytic
   solution to simple dynamic model :math:`\to` PID), chapter on
   dynamics)

-  Chris Atkeson’s “Kinematics, Dynamic Systems, and Control”
   <http://www.cs.cmu.edu/ cga/kdc/>

   (uses Schaal’s slides and LaValle’s book, useful: slides on 3d
   kinematics <http://www.cs.cmu.edu/ cga/kdc-10/ewhitman1.pptx>)

-  CMU lecture “introduction to robotics”
   <http://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/current/>

   (useful: PID control, simple BUGs algorithms for motion planning,
   non-holonomic constraints)

-  *Springer Handbook of Robotics, Bruno Siciliano, Oussama Khatib*
   <http://link.springer.com/book/10.1007/978-3-319-32552-1>

-  LaValle’s *Planning Algorithms* <http://planning.cs.uiuc.edu/>

Coding Getting Started
~~~~~~~~~~~~~~~~~~~~~~

Please follow the instructions at github/robotics-course
[<https://marctoussaint.github.io/robotics-course/>] for setting up the
python package. This includes a series of tutorials, which can also be
downloaded here [<https://github.com/MarcToussaint/rai-tutorials>].

Scene & Robot Description
-------------------------

Generally speaking, a scene is a collection of objects (including robot
parts). We typically assume objects to be rigid bodies with fixed shape
– which clearly is a simplification relative to real world. More about
this below, in section `1.14.1 <#secShapes>`__.

However, formally we define a scene as a collection of **frames**, which
is short for coordinate frames. We can think of these frames as oriented
locations in 3D space – and various things can be associated to these
frames. If a rigid shape and mass is associated to a frame, then it
makes a typical rigid body. But frames can also be associated to robot
joint locations or virtual landmarks.

Transformations
~~~~~~~~~~~~~~~

Let :math:`i=1,..,m` enumerate :math:`m` frames in a scene. Each frame
has a **pose** :math:`X_i\in SE(3)`, where
:math:`SE(3) = {\mathbb{R}}^3 \times SO(3)` is the group of 3D
transformations, namely the cross-product of translations and rotations.
We always assume a world origin to be defined and use the word *pose*
specifically for the transformation from world origin to the object
frame.

Transformations in :math:`A\in SE(3)` are tuples :math:`A = (t, r)`,
where :math:`t\in{\mathbb{R}}^3` is a translation and :math:`r\in SO(3)`
a rotation – see Appendix `1.10 <#appTransforms>`__ for more details.
Rotations can be represented as matrix :math:`R` (see the Maths script
on properties of rotation matrices), and a pose as the :math:`4\times 4`
homogeneous transform
:math:`\left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right)`.
However, more commonly in code we represent rotations as a 4D quaternion
:math:`r\in{\mathbb{R}}^4` with unit length :math:`|r| = 1`. I always
use the convention :math:`r=(r_0,\bar r)`, where the first entry
:math:`r_0 = \cos(\theta/2)` relates to the total rotation angle
:math:`\theta`, and the last three entries
:math:`\bar r = \sin(\theta/2)~ \underline w` relate to the unit length
rotation axis :math:`\underline w`.

Euler angles and the scaled rotation vector are alternative rotation
representations – but never use them. The appendix
`1.10 <#appTransforms>`__ introduces to all these representations and
derives conversion equations to relate them.

The illustrates how you can manually define frames in a configuration
and set absolute or relative transformations.

Coordinates and Composition of Transformations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|image|

[figTransforms] Composition of transforms.

Consider Fig. \ `[figTransforms] <#figTransforms>`__, were we have three
frames :math:`1,2,3` in addition to the world origin frame :math:`W`.
Each frame has a global pose :math:`X_1, X_2, X_3`, and relative
transforms :math:`Q_{W\to 1}, Q_{1\to 2}, Q_{2\to 3}`. We have

.. math::

   \begin{aligned}
   X_1 &= Q_{W\to 1} \\
   X_2 &= Q_{W\to 1} \circ Q_{1\to2} \\
   X_3 &= Q_{W\to 1} \circ Q_{1\to2} \circ Q_{1\to3} ~.\end{aligned}

Note that when composing relative transforms, we concatenate (append)
them *on the right*! Intuitively, this describes a concatenation of
turtle commands, where a turtle is initially placed on the world origin,
then translates, then rotations, then translates *relative to its own
pose*, then rotations *relative to its own pose*, etc, and ends up in
pose :math:`X_3`.

Now consider the position of a point in 3D space. It can be given in
world coordinates :math:`x^W`, but also in relative coordinates
:math:`x^1, x^2, x^3`. We have

.. math::

   \begin{aligned}
   x^W &= Q_{W\to 1}~ Q_{1\to2}~ Q_{1\to3}~ x^3 = X_3~ x^3 ~.\end{aligned}

Now you might want to ask: “does :math:`Q_{1\to 2}` describe the forward
or the backward transformation from frame :math:`1` to frame :math:`2`?”
But this question is somewhat ill-posed. The situation is:

-  :math:`Q_{1\to 2}` describes the translation and rotation of *frame*
   :math:`2` *relative* to :math:`1`. So you may call it the “forward
   FRAME transformation”.

-  :math:`Q_{1\to 2}` describes the coordinate transformation from
   :math:`x^2` to :math:`x^1 = Q_{1\to 2} x^2`. So you may call it the
   “backward COORDINATE transformation”.

In the view of fundamental linear algebra, this should not surprise as
basis vectors transform *covariant*, while coordinates transform
*contra-variant*. The appendix `1.10.2.1 <#secTransNotation>`__ explains
this again in more detail and with an explicit example.

Scene Tree or Forest
~~~~~~~~~~~~~~~~~~~~

Scenes are typically represented as trees, with the world origin as a
root, and the pose of children specified by a *relative* transformation
from the parent. For instance, a scene with a book on a table on the
ground on the world, would have four frames with poses
:math:`X_0, X_1, X_2, X_3` (of the world, ground, table, book), but the
scene would typically be represented by relative transforms
:math:`Q_1, Q_2, Q_3` such that

.. math:: X_i = X_{i{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}} \circ Q_i ~.

Note that every frame can only have a single parent, and we can
abbreviate the notation :math:`Q_i \equiv Q_{\text{parent}(i)\to i}`.

Scenes can also be a forest of frames, where some frames have no parent
and their pose :math:`X_i` must be specified, while for non-roots the
relative transform :math:`Q_i` is specified. We usually only talk about
trees, but include meaning forests.

The also demonstrates how to define a fram a *child* of another, thereby
defining a frame tree. Instead of the absolute pose ``X``, you typically
specify the relative transformation ``Q`` for such a child frame.

Kinematics
----------

Robots as Parameterized Trees
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The key thing in robotics is that some relative transforms (between
robot links) are “motorized” and can be moved. Formally, this means that
*some* of the relative transforms :math:`Q_i` in our scene have
**degrees of freedom** (dof) :math:`q_i \in {\mathbb{R}}^{d_i}`.

For typical robots (with hinge or linear joints) each :math:`q_i` is
just a single number (the joint dimensionality :math:`d_i=1`). E.g., a
**hinge** joint around the (local) :math:`x`-axis has a single dof
:math:`q_i\in{\mathbb{R}}` that parameterizes the relative transform

.. math::

   \begin{aligned}
   Q_i(q_i) =  \left(\begin{array}{cccc}
   1 & 0 & 0 & 0 \\
   0 & \cos(q_i) & -\sin(q) & 0 \\
   0 &  \sin(q_i) & \cos(q) & 0 \\
   0 & 0 & 0 & 1\end{array}\right)  ~.\end{aligned}

And a **prismatic** (or translational) joint along the (local)
:math:`x`-axis parameterizes

.. math::

   \begin{aligned}
   Q_i(q_i) =  \left(\begin{array}{cccc}
   1 & 0 & 0 & q \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1\end{array}\right)  ~.\end{aligned}

Other joint types (universal, cylindrical) are less common.

A bit special are **ball (spherical) joints**: They parameterize
arbitrary rotations within :math:`Q_i` – in principle they could be
described as having 3 dofs (as the Lie group :math:`SO(3)` is a 3D
manifold), however, in code it is practice to again use quaternions to
parameterize rotations, which means :math:`q_i\in{\mathbb{R}}^4` for
ball joints. However, note that this is an over parameterization: If
:math:`q_i` is externally “set” by a user or some algorithm, it may not
(exactly) be normalized but :math:`Q_i(q_i)` is defined to be the proper
rotation that corresponds to the quaternion :math:`q_i/|q_i|`. Note that
if a user or algorithms sets such a quaternion parameter to zero, that’s
a singularity and strict error.

In the , when a joint is define for the first time, play around with
alternative joint types, e.g. a ``quatBall``. The tutorial also lists
which joint types are pre-defined.

In the scene tree, some of the relative transforms :math:`Q_i` are
parameterized by dofs, :math:`Q_i(q_i)`. Note that
:math:`X_\text{parent$(i)$}` is the **joint origin**, i.e., determines
the location and orientation of the joint axis, while
:math:`X_i = X_\text{parent$(i)$} Q_i` is the **joint (output) frame**.
In a robot structure one typically has chains of alternating rigid and
parameterized transforms, e.g.,

a rigid transform :math:`Q_{\pi(i)}` from world into the origin of joint
:math:`i`

a parameterized transform :math:`Q_i(q_i)` representing the joint motion
(We call this one the *joint frame*, as it hosts the joint dofs.)

a rigid transform :math:`Q_{i \to \pi(j)}` from the output of :math:`i`
into the origin of a following joint :math:`j`

a parameterized transform :math:`Q_j(q_j)`

etc

There is a minimalistic convention of describing robot structures,
called Denavit-Hartenberg convention. These describe the rigid
transformations between joints using only 4 numbers instead of 6 (which
pre-determines the zero calibration as well as the “lateral” positioning
of the following joint origin). But there is no need to use this
convention and the above notation is conceptually cleaner and leads to
intuitive, freely user-defined joint origins.

In the you find a section on interactively editing a scene description
file ``mini.g``. Using this you can try to invent your own robot and
environment. The tutorial also shows how to load pre-define robot
models. The appendix `[secConfigFiles] <#secConfigFiles>`__ provides a
more formal specification of the yaml-style file syntax.

Forward Kinematics
~~~~~~~~~~~~~~~~~~

We use the word **configuration** for an “articulated scene”, i.e.,
where some relative transforms :math:`Q_i(q_i)` are parameterized by
dofs :math:`q_i \in {\mathbb{R}}^{d_i}` (and also other dofs such as
forces or timings might be represented). A configuration can include
multiple robots – from our perspective there is no difference between
one or multiple robots. It’s just a parameterized forest of frames.

We define the **joint vector** :math:`q\in{\mathbb{R}}^n` to be the
stacking of all dofs :math:`q_i` (all dofs of a configuration). Given
the joint vector, we can forward chain all relative transformations in
the scene and thereby compute the absolute pose :math:`X_i(q)` of every
frame as a function of :math:`q`.

This function :math:`q \mapsto X_i(q)` is the core of **forward
kinematics**. It describes how the joint vector :math:`q` determines the
pose of all frames in the configuration.

The precise definition of the term **forward kinematics** varies across
textbooks. I find the most concise definition to be the mapping from all
dofs :math:`q` to the full configuration state
:math:`\{X_i(q)\}_{i=1}^m`, which so far we described in terms of all
frame poses. This definition is consistent with the formal description
of *kinematics* as the theory of possible motions of a system
configuration (see `1.14.2 <#secKinematics>`__).

But in practice, the word forward kinematics is often used simply as the
mapping from :math:`q` to one particular “feature” of the configuration.
For instance, if :math:`X_i(q)=(t_i(q),r_i(q))` is the pose of some
frame :math:`i` (e.g. the “end-effector”), forward kinematics can
describe the mapping

-  :math:`q\mapsto t_i(q)`   to the position of frame :math:`i`

-  :math:`q\mapsto r_i(q) \cdot \textbf{e}_x`   to the :math:`x`-axis of
   frame :math:`i` (where :math:`\textbf{e}_x = (1,0,0)^{\!\top\!}`).

-  :math:`q\mapsto X_i(q) p`   to the world coordinate of a point
   attached to frame :math:`i` with fixed relative offset :math:`p`.

Each of these are 3-dimensional features. Let introduce a more formal
notation for these three basic features:

.. math::

   \begin{aligned}
   q \mapsto \phi^{\textsf{pos}}_{i,p}(q) &= X_i(q)~ p \quad\in {\mathbb{R}}^3 ~, \\
   q \mapsto \phi^{\textsf{vec}}_{i,v}(q) &= r_i(q) \cdot v \quad\in {\mathbb{R}}^3 ~, \\
   q \mapsto \phi^{\textsf{quat}}_{i}(q) &= r_i(q) \quad\in {\mathbb{R}}^4 ~,\end{aligned}

where :math:`\phi^{\textsf{pos}}_{i,p}(q)` is the (world) position of a
point attached to frame :math:`i` with relative offset :math:`p`,
:math:`\phi^{\textsf{vec}}_{i,v}(q)` is the world coordinates of a
vector :math:`v` attached to frame :math:`i`, and
:math:`\phi^{\textsf{quat}}_{i}(q)` is the 4D quaternion orientation of
frame :math:`i`. From these three, many others features can be derived.

E.g., also the :math:`3\times 3` rotation matrix is a useful basic
feature (as it is often used in equations). We can easily construct it
by concatenating columns, :math:`\phi^{\textsf{rot}}_i =
(\phi^{\textsf{vec}}_{i,e_x}, \phi^{\textsf{vec}}_{i,e_y}, \phi^{\textsf{vec}}_{i,e_z}) \in {\mathbb{R}}^{3\times
3}` for basis vectors :math:`e_x,e_y,e_z` of frame :math:`i`. (Note that
the Jacobian (defined later) of this is a :math:`3\times 3 \times n`
tensor.)

The output space of the kinematic map is also called **task space**.
However, I often just call it **kinematic feature**.

The illustrates how you get the joint vector :math:`q` and set it. This
way you can animate the configuration. Also the positions and
orientations of all frames can be queried directly – realizing the most
basic kind of forward kinematics.

Jacobians
~~~~~~~~~

We will use kinematic features :math:`\phi` to formulate differentiable
constraint and optimization problem. Therefore, we assume all kinematic
features :math:`\phi` are differentiable and we can efficiently compute
the **Jacobian**

.. math::

   \begin{aligned}
   J(q) = \frac{\partial}{\partial q}\phi(q) ~.\end{aligned}

If :math:`y = \phi(q)`, then this Jacobian tells us how a velocity
:math:`\dot q` in joint space implies a velocity :math:`\dot y` in task
space,

.. math::

   \begin{aligned}
   \dot y = J(q) \dot q ~.\end{aligned}

Recall that the forward kinematics is essentially implemented by forward
chaining the relative transforms :math:`Q_i`. If we use an
auto-differentiable programming language for this, we’d directly have
the Jacobians. However, the Jacobians can also directly be expressed
analytically and their computation turns out simpler and more efficient
than the forward chaining itself. To implement a kinematic engine we
essentially need to figure out how the different joint types contribute
to the Jacobians of the three basic features above. This is covered by
considering the following cases:

Rotational Joint
^^^^^^^^^^^^^^^^

Consider that somewhere on the path from world to frame :math:`i` there
is a rotational (hinge) joint :math:`j` positioned at :math:`p_j` and
with unit axis vector :math:`a_j` (both in world coordinates). Now
consider a point attached to frame :math:`i` at world coordinate
:math:`p`. (Note that we needed forward kinematics to determine
:math:`p_j, a_j`, and :math:`p`.) Then the velocity :math:`\dot p`
relates to the joint angle velocity :math:`\dot q_j` by

.. math:: \dot p = [a_j \times (p - p_j)]~ \dot q_j ~.

Now assume a vector :math:`v` attached to frame :math:`i`. Its velocity
is

.. math:: \dot v = [a_j \times v]~ \dot q_j = [-{\text{skew}}(v)~ a_j]~ \dot q_j ~.

Now consider the quaternion :math:`r_i` of frame :math:`i`. Its velocity
(much less obvious, see appendix Eq. (\ `[eqQuatVel] <#eqQuatVel>`__))
is

.. math:: \dot r_i = {\frac{1}{2}}[(0,a_j)\circ r_i]~ \dot q_j ~.

Recall that :math:`q\in{\mathbb{R}}^n` is the full joint vector. Let
:math:`j` be the dof index of our rotational joint such that
:math:`q_j \in {\mathbb{R}}` is the scalar joint angle. Further, let
:math:`p_j,a_j` be the joint position and axis, and :math:`p` a world
query point. We define two matrices that are zero except for the
provided columns:

.. math::

   \begin{aligned}
   J^{\textsf{ang}}\in {\mathbb{R}}^{3 \times n} \quad\text{with}\quad &J^{\textsf{ang}}_{:,j} = a_j ~, \\
   J^{\textsf{pos}}(p) \in {\mathbb{R}}^{3 \times n} \quad\text{with}\quad &J^{\textsf{pos}}_{:,j} = a_j \times (p - p_j) ~.\end{aligned}

With these two matrices we can rewrite the above equations as

.. math::

   \begin{aligned}
   \dot p &= J^{\textsf{pos}}(p)~ \dot q \\
   \dot v &= [-{\text{skew}}(v)~ J^{\textsf{ang}}(p)]~ \dot q \\
   \dot r &= {\frac{1}{2}}[\text{Skew}(r)~ \bar J^{\textsf{ang}}(p)]~ \dot q \quad\text{where}\quad \text{Skew}(w,x,y,z) =
    \left(\begin{array}{cccc}
      +w & -x & -y & -z \\
      +x & +w & +z & -y \\
      +y & -z & +w & +x \\
      +z & +y & -x & +w\end{array}\right)  ~, \label{eqQuatRate}\end{aligned}

where by convention the cross-product :math:`[A\times v]` for a
:math:`3\times n` matrix with a 3-vector takes the cross-products
*row-wise* (could perhaps better be written :math:`[-v\times A]`). The
last equation is derived in the appendix with
Eq. (\ `[eqQuatVel] <#eqQuatVel>`__), where we discuss how an angular
velocity translates to a quaternion velocity. The bar in
:math:`\bar J^{\textsf{ang}}` makes this a :math:`4\times n` matrix by
inserting a zero top row (analogous to :math:`(0,w)` in
(`[eqQuatVel] <#eqQuatVel>`__)). The :math:`\text{Skew}` is an unusual
definition of a skew matrix for quaternions, so that quaternion
multiplication :math:`a \circ b` can be written linearly as
:math:`\text{Skew}(b)~ a`.

Now, if in our scene tree we have more than one rotational joint between
world and frame :math:`i`, each of these joints simply contribute
non-zero columns to our basic matrices
:math:`J^{\textsf{ang}}, J^{\textsf{pos}}(p)`. So this is the core of
what we have to implement for rotational joints.

Translational Joint
^^^^^^^^^^^^^^^^^^^

A translational (prismatic) joint on the path from world to frame
:math:`i` also contribute a column to the basic matrix
:math:`J^{\textsf{pos}}(p)`, but contributes notion to
:math:`J^{\textsf{ang}}` (as it does not imply rotational velocity in
the sub-branch). Specifically, let :math:`a_j` the translational axis of
the joint with dof index :math:`j`, then it simply contributes a column

.. math::

   \begin{aligned}
   J^{\textsf{pos}}_{:,j} = a_j ~.\end{aligned}

That’s it for translational joints.

Quaternion Joint
^^^^^^^^^^^^^^^^

Trickier, but important for ball and free joints is to also know how a
quaternion joint contributes columns to :math:`J^{\textsf{ang}}` and
:math:`J^{\textsf{pos}}(p)`. Modifying a quaternion parameterization
:math:`q_j\in{\mathbb{R}}^4` of a relative transform :math:`Q_j(q_j)`
implies in some way a rotational velocity down the branch. So the effect
should be similar to a rotational joint, but without fixed axis and
modulated by the normalization of :math:`q_j`. The solution is derived
in the appendix with Eq. (\ `[eqQuatJac] <#eqQuatJac>`__) and summarized
here: Let :math:`X_j` be the *output* pose of the quaternion joint.
(Yes, output!) And let :math:`R_j` be the :math:`3\times 3` rotation
matrix for the world pose :math:`X_j`, and let
:math:`r_j \in {\mathbb{R}}^4` be the quaternion of the *relative* joint
transform :math:`Q_j`. Then

.. math::

   \begin{aligned}
   \label{eqQuatJoint1}
   J^{\textsf{ang}}_{:,j} = \frac{1}{|q|} R_j J(r_j) ~,\quad\text{where}\quad
   J(r)_{:,k} &= -2 (e_k \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1})_{1:3} ~.\end{aligned}

Here, :math:`e_i` for :math:`k=0,..,3` are the unit quaternions and the
matrix :math:`J(r)\in{\mathbb{R}}{3 \times 4}` describes how a variation
of a quaternion :math:`r` induces a 3D rotation vector relative to the
*output* space of :math:`r`. I call this the quaternion Jacobian. The
derivation is found in the appendix when discussion how a quaternion
velocity implies and angular velocity. The multiplication with
:math:`R_j` transforms this rotation vector to world coordinates. The
division by :math:`|q_j|` accounts when the dof :math:`q_j` is not
(exactly) normalized.

As we figured out the angular vector induced by a variation of a
quaternion joint, this also defines the column it contributes to the
positional Jacobian:

.. math::

   \begin{aligned}
   J^{\textsf{pos}}_{:,j}(p) = [\frac{1}{|q|} R_j J(r_j)] \times (p - p_j) ~,\end{aligned}

where :math:`p_j` is the position of the quaternion joint.

Note how differently we treat the quaternion :math:`q_j` as a joint
parameterization :math:`Q_j(q_j)` and the quaternion :math:`r_i` as a
kinematic (“output”) feature of frame :math:`i`. For instance, we can
have the Jacobian of the quaternion :math:`r_i` w.r.t. the quaternion
joint parameterization :math:`q_j`, by inserting
(`[eqQuatJoint1] <#eqQuatJoint1>`__) into
(`[eqQuatRate] <#eqQuatRate>`__). And even if all other transformation
in the scene are identities and the output quaternion :math:`r_i` is
“essentially identical” to the joint quaternion :math:`q_j`, the
Jacobian is still not exactly identity, as it accounts for normalization
(and potential flip of sign).

General Concept of Differentiable Features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the previous sections we focussed on the 3 mappings
:math:`\phi^{\textsf{pos}}_{i,p}(q), \phi^{\textsf{vec}}_{i,v}(q), \phi^{\textsf{quat}}_{i}(q)`.
The Jacobians of these are given via :math:`J^{\textsf{pos}}_{:,j}(p)`
and :math:`J^{\textsf{ang}}_{:,j}(p)`. If these are given
(e.g. implemented by an efficient core kinematics engine), then many
other features can be computed based on them.

We assume a single configuration :math:`q`, or a whole set of
configurations :math:`\{q_1,..,q_T\}`, with each
:math:`q_i \in\mathbb{R}` the DOFs of that configuration.

In general, a (0-order) **feature** :math:`\phi` is a differentiable
mapping

.. math:: \phi: q \mapsto \mathbb{R}^D

of a single configuration into some :math:`D`-dimensional space.

The introduces to features that are readily implemented in the rai code.

(In C++, new features can be implemented by overloading the abstract
Feature class. Implementing new features is typically done by first
evaluating existing features and then “forward chaining” the computation
of the new feature – quite similar to how models are defined in pyTorch
or similar autodiff frameworks. The C++ code uses autodiff (which
*forward* chains Jacobians directly at computation) for most features.)

When using features in code, one can additionally specify a ``target``
and ``scale``, which defines a subsequent linear transformation:

.. math:: \phi(q) \gets \texttt{scale} \cdot (\phi(q) - \texttt{target})

Note that the scale can be a matrix, which projects the feature. E.g.,
if you want to define a 2D feature which is the :math:`xy`-position of
frame :math:`i`, then you can use a matrix
:math:`\texttt{scale}= \left(\begin{array}{ccc}1 & 0 & 0 \\ 0 & 1 & 0\end{array}\right)`.

Further, a feature can also be of higher order, which by default means a
finite difference of a 0-order features. In general, a higher-order
features is a differentiable mapping

.. math:: \phi: (q_0,q_1,..,q_k) \mapsto \mathbb{R}^D

of a :math:`(k+1)`-tuple of configurations to a :math:`D`-dimensional
space. This is typically used in the context of **path configurations**,
which is a sequence of configurations used in path optimization.

Given any 0-order feature :math:`\phi`, by default that defines its 1st
and 2st order feature as

.. math:: \phi(q_0,q_1) = \frac{1}{\tau}(\phi(q_1) - \phi(q_0))

and

.. math:: \phi(q_0,q_1,q_2) = \frac{1}{\tau^2}(\phi(q_2) - 2 \phi(q_1) + \phi(q_0)) ~,

which are the finite difference approximations of the feature’s velocity
and acceleration. However, one can also directly implement higher-order
features, e.g. to represent dynamics constraints, or more elaborate
acceleration/torque cost features.

Summary: Implementing a Kinematic Engine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The above provides all essentials necessary to implement a rather
general kinematic engine. To summarize:

-  Represent a scene configuration as a tree of frames, where for each
   frame we store the absolute pose :math:`X` and relative transform
   :math:`Q`. We also annotate which relative transforms :math:`Q` have
   dofs and how many. We need to maintain an index mapping that tells us
   which entries :math:`q_j` of the full joint vector parameterize a
   given relative transformation :math:`Q_j(q_j)` (essentially mapping
   between :math:`q`-indices and frame indices).

-  An efficient implementation of forward chaining transformations:
   Given the absolute poses :math:`X` of all root frames and all
   relative transforms :math:`Q`, implement an efficient algorithm to
   forward chain transformations to ensure any :math:`X_i`. Do this
   lazily on demand: Only when an absolute frame :math:`X_i` is actually
   queried call this forward chaining for this :math:`X_i` only.

-  An efficient implementation of the matrices :math:`J^{\textsf{pos}}`
   and :math:`J^{\textsf{ang}}`, which, for any query frame :math:`i`,
   determines which joints are on the path from :math:`i` to a root
   frame and for each of these joints contributes the corresponding
   columns to :math:`J^{\textsf{pos}}` and :math:`J^{\textsf{ang}}`. To
   account for large systems (esp. path configurations, see below)
   matrices should be returned in sparse format.

Based on this, one provides more convenient user functions that allow to
query kinematic features for any frame :math:`i`, including the pose
:math:`X_i`, and on demand also provide the Jacobian of that feature.

Inverse Kinematics
~~~~~~~~~~~~~~~~~~

|image|

We can “puppeteer” a robot by defining optimization problems with task
space constraints and solve for the joint state.

We introduced forward kinematics as a mapping from an
:math:`n`-dimensional joint vector :math:`q\in{\mathbb{R}}^n` to some
:math:`d`-dimensional kinematic feature
:math:`y=\phi(q) \in{\mathbb{R}}^d`. Inverse kinematics roughly means to
invert this mapping, i.e., given a desired target :math:`y^*` in task
space, find a joint vector :math:`q` such that :math:`\phi(q) = y^*`. As
often :math:`n>d`, the inversion is under-specified (leading to what is
called “redundancy”). But just as the pseudo-inverse of a linear
transformation addresses this, we can generalize this to a non-linear
:math:`\phi` – namely in an optimality formulation.

Given :math:`\phi` and a target :math:`y^*`, a good option is to define
**inverse kinematics** as the non-linear mathematical program (NLP)

.. math::

   \begin{aligned}
   \label{eqIKNLP}
   q^* = \argmin_q f(q) ~~\text{s.t.}~~\phi(q) = y^* ~.\end{aligned}

The cost term :math:`f(q)` is called *regularization* and indicates a
preference among all solutions that satisfy :math:`\phi(q) = y`. One
might typically choose it as a squared distance
:math:`f(q) = |\!|q-q_0|\!|^2_W` to some “default” :math:`q_0`, which
could be the homing state of a robot or its current state.

In practice, I recommend always using a proper NLP solver to solve
inverse kinematics. As discussing optimization is beyond this script we
are here already done with describing inverse kinematics! It is “nothing
more” than defining a constraint problem of the sort
(`[eqIKNLP] <#eqIKNLP>`__) and passing it to a solver. In the coding
part below I will discuss the richness in options to define such
constraint problems with our differentiable features.

Only for educational purpose we will also derive the classical
pseudo-inverse Jacobian solution to IK below.

Building an NLP from features
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Eq. (\ `[eqIKNLP] <#eqIKNLP>`__) describes IK as an NLP. Appendix
`1.12.1 <#secNLP>`__ provides a technical reference of how we define
NLPs mathematically and in code. Essentially, an NLP is specified by
*adding objectives*, where each objective is given by a feature function
:math:`\phi_i` and an indicator :math:`\varrho_i` that defines whether
the feature contributes a linear cost (``f``), sum-of-squares cost
(``sos``), equality constraint (``eq``), or inequality constraint
(``ineq``) to the NLP.

The illustrates how an Inverse Kinematics problem can be specified as
NLP. The core is the ``addObjective`` method, which adds a kinematic
feature (optimally with transformed by scaling and target) as a cost or
constraint (depending on the ``f``, ``sos``, ``eq``, or
``ineq``\ indicator) to the NLP.

Classical Derivation of Pseudo-Inverse Jacobian Solution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

I strongly recommend using an NLP solver and general constraint and cost
formulations to tackle IK problems – and you can skip over this section.
However, for completeness I provide here also the basic derivation of
classical pseudo-inverse Jacobian solutions.

Pseudo-inverse Jacobian.
''''''''''''''''''''''''

We first simplify the problem to minimize

.. math::

   \begin{aligned}
   \label{eqSoft}
   f(q) = |\!|\phi(q) - y^*|\!|^2_C + |\!|q-q_0|\!|^2_W ~.\end{aligned}

Instead of exactly ensuring :math:`\phi(q) = y^*`, this only minimizes a
penalty :math:`|\!|\phi(q) - y^*|\!|^2_C`. Here :math:`C` is the norm’s
metric, i.e., :math:`|\!|v|\!|^2_C = v^{\!\top\!}C v`, but you may
instead simply assume :math:`C` is a scalar. For finite :math:`C` and
:math:`W` this approximate formulation might be undesirable. But later
we will actually be able to investigate the limit :math:`C\to\infty`.

Since this problem is a least squares problem, the canonical approach is
Gauss-Newton. The gradient, approximate Hessian, and Gauss-Newton step
are

.. math::

   \begin{aligned}
   \frac{\partial}{\partial q} f(q)
   &= 2 (\phi(q)-y^*)^{\!\top\!}C J + 2 (q-q_0)^{\!\top\!}W = {\nabla_{\!\!f}}(q)^{\!\top\!}\\
   {\nabla_{\!\!f}^2}(q)
   &\approx 2 (J^{\!\top\!}C J + W) \\
   \delta(q)
   &= - [{\nabla_{\!\!f}^2}(q)]^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} {\nabla_{\!\!f}}(q) = (J^{\!\top\!}C J + W)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} [J^{\!\top\!}C (\phi(q)-y^*) + W (q-q_0) ]\end{aligned}

With some identities, this can be rewritten as

.. math::

   \begin{aligned}
   \delta(q)
   &= J^\sharp (y^* - \phi(q)) + (I - J^\sharp J)~ (q_0 - q) \label{eqIK} \\
   J^\sharp
   &= (J^{\!\top\!}C J + W)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} J^{\!\top\!}C = W^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} J^{\!\top\!}(J W^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} J^{\!\top\!}+ C^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1})^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}
    \text{(Woodbury identity)}\end{aligned}

The matrix :math:`J^\sharp` is also called (regularized) pseudo-inverse
of :math:`J`. In its second form (RHS of Woodbury), we can take the hard
limit :math:`C\to\infty`, where
:math:`J^\sharp \to W^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} J^{\!\top\!}(J W^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} J^{\!\top\!})^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}`
or, for :math:`W={\rm\bf I}`,
:math:`J^\sharp \to J^{\!\top\!}(J J^{\!\top\!})^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}`.

Eq. (\ `[eqIK] <#eqIK>`__) says that, to jump to the (approx.)
Gauss-Newton optimum, we should make a step :math:`\delta` in joint
space proportional to the error :math:`(y^*-\phi(q))` in task space, and
(optionally) combined with a homing step towards :math:`q_0` projected
to the task null space via the projection :math:`(I - J^\sharp J)`.

Performing a single step :math:`\delta` is approximate due to the
non-linearity of :math:`\phi`. To solve inverse kinematics exactly we
have to iterate Gauss-Newton steps. If lucky, we can use full stepsizes
(:math:`\alpha= 1` in the speak of line search) and iterate
:math:`q_{k{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}} \gets q_k + \delta(q_k)`
until convergence, and will have an exact IK solution. If :math:`\phi`
is very non-linear, we may have to do line searches along the step
directions to ensure convergence. If :math:`\phi` is non-convex, we may
converge to a local optimum that depends on the initialization.

On the fly IK.
''''''''''''''

Inverse kinematics is sometimes being used to generate robot motion on
the fly. In a sense, rather than letting an optimization algorithm find
an IK solution and then start moving the robot to it (we we’ll do it
below), you let the robot directly move (generate a smooth path) towards
an IK solution. This is heuristic, and I eventually don’t recommend it.
But it’s standard practice, so let’s mention it:

Let the robot be in state :math:`q`, and we have a task space target
:math:`y^*`. We may compute a desired robot motion

.. math::

   \begin{aligned}
   \dot q = \alpha\Big[ J^\sharp (y^* - \phi(q)) + (I - J^\sharp J) (q_0 - q) \Big] ~.\end{aligned}

In a sense, this mimics performing (integrating over time) infinitesimal
Gauss-Newton steps towards the IK solution. Often the regularization
:math:`(I - J^\sharp J) (q_0 - q)` is also dropped, which is the same as
saying :math:`q_0 = q`, i.e., you always set the homing state
:math:`q_0` to be the current state :math:`q`, adapting it on the fly.
Doing this, you will loose a precise definition of where you’ll
eventually converge to – and sometimes this leads to undesired *drift in
nullspace*. All not recommended.

Singularity.
''''''''''''

The limit :math:`C\to\infty` mentioned above is only robust when
:math:`\det (J
J^{\!\top\!}) > 0`, or equivalently, when :math:`J` has full rank
(namely rank :math:`d`). :math:`J` is called singular otherwise, and the
pseudo inverse :math:`J^\sharp` is ill-defined.

Intuitively this means that, in state :math:`q`, certain task space
directions cannot be generated, i.e., no motion in these task space
directions is possible. A stretched arm that cannot extend further is a
typical example.

In the original NLP formulation, this corresponds to the case where
:math:`\phi(q) = y^*` is simply infeasible, and a proper NLP-solver
should return this information.

The soft problem formulation (`[eqSoft] <#eqSoft>`__), where :math:`C`
is finite (not :math:`\infty`) is one way to address a singularity: For
finite :math:`C`, :math:`J^\sharp` is well defined and defines steps
towards a optimal solution of the trade-off problem
(`[eqSoft] <#eqSoft>`__). This is also called **regularized IK** or
**singularity-robust IK**. But it only leads to an approximate IK
solution.

.. _appTransforms:

3D Transformations, Rotations, Quaternions
------------------------------------------

Rotations
~~~~~~~~~

There are many ways to represent rotations in :math:`SO(3)`. We restrict
ourselves to three basic ones: rotation matrix, rotation vector, and
quaternion. The rotation vector is also the most natural representation
for a “rotation velocity” (angular velocities). Euler angles or
raw-pitch-roll are an alternative, but they have singularities and I
don’t recommend using them in practice.

A rotation matrix
   is a matrix :math:`R\in{\mathbb{R}}^{3\times3}` which is orthonormal
   (columns and rows are orthogonal unit vectors, implying determinant
   1). While a :math:`3\times3` matrix has 9 degrees of freedom (DoFs),
   the constraint of orthogonality and determinant 1 constraints this:
   The set of rotation matrices has only 3 DoFs (:math:`\sim` the local
   Lie algebra is 3-dim).

   -  The application of :math:`R` on a vector :math:`x` is simply the
      matrix-vector product :math:`R x`.

   -  Concatenation of two rotations :math:`R_1` and :math:`R_2` is the
      normal matrix-matrix product :math:`R_1 R_2`.

   -  Inversion is the transpose,
      :math:`R^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} = R^{\!\top\!}`.

A rotation vector
   is an unconstrained vector :math:`w\in{\mathbb{R}}^3`. The vector’s
   direction :math:`\underline w = \frac{w}{|w|}` determines the
   rotation axis, the vector’s length :math:`|w|=\theta` determines the
   rotation angle (in radians, using the right thumb convention).

   -  The application of a rotation described by
      :math:`w\in{\mathbb{R}}^3` on a vector :math:`x\in{\mathbb{R}}^3`
      is given as (Rodrigues’ formula)

      .. math::

         \begin{aligned}
         w \cdot x
          &= \cos\theta~ x
           + \sin\theta~ (\underline w\times x)
           + (1-\cos\theta)~ \underline w(\underline w^{\!\top\!}x)\end{aligned}

      where :math:`\theta=|w|` is the rotation angle and
      :math:`\underline w=w/\theta` the unit length rotation axis.

   -  The inverse rotation is described by the negative of the rotation
      vector.

   -  Concatenation is non-trivial in this representation and we don’t
      discuss it here. In practice, a rotation vector is first converted
      to a rotation matrix or quaternion.

   -  Conversion to a matrix: For every vector
      :math:`w\in{\mathbb{R}}^3` we define its skew symmetric matrix as

      .. math::

         \begin{aligned}
         \hat w = \text{skew}(w) =  \left(\begin{array}{ccc}0 & -w_3 & w_2 \\ w_3 & 0 & -w_1 \\-w_2 & w_1 & 0\end{array}\right)  ~.\end{aligned}

      Note that such skew-symmetric matrices are related to the cross
      product: :math:`w \times v = \hat w~ v`, where the cross product
      is rewritten as a matrix product. The rotation matrix :math:`R(w)`
      that corresponds to a given rotation vector :math:`w` is:

      .. math::

         \begin{aligned}
         \label{eqRodriguez}
         R(w)
          &= \exp(\hat w) \\
          &= \cos\theta~ I + \sin\theta~ \hat w/\theta+ (1-\cos\theta)~ w w^{\!\top\!}/\theta^2\end{aligned}

      The :math:`\exp` function is called exponential map (generating a
      group element (=rotation matrix) via an element of the Lie algebra
      (=skew matrix)). The other equation is called Rodrigues’ equation:
      the first term is a diagonal matrix (:math:`I` is the 3D identity
      matrix), the second terms the skew symmetric part, the last term
      the symmetric part (:math:`w
      w^{\!\top\!}` is also called outer product).

Angular velocity & derivative of a rotation matrix:
   We represent angular velocities by a vector
   :math:`w\in{\mathbb{R}}^3`, the direction :math:`\underline w`
   determines the rotation axis, the length :math:`|w|` is the rotation
   velocity (in radians per second). When a body’s orientation at time
   :math:`t` is described by a rotation matrix :math:`R(t)` and the
   body’s angular velocity is :math:`w`, then

   .. math::

      \begin{aligned}
      \label{eqDotR}
      \dot R(t) = \hat w~ R(t)~.\end{aligned}

   (That’s intuitive to see for a rotation about the :math:`x`-axis with
   velocity 1.) Some insights from this relation: Since :math:`R(t)`
   must always be a rotation matrix (fulfill orthogonality and
   determinant 1), its derivative :math:`\dot R(t)` must also fulfill
   certain constraints; in particular it can only live in a
   3-dimensional sub-space. It turns out that the derivative
   :math:`\dot R` of a rotation matrix :math:`R` must always be a skew
   symmetric matrix :math:`\hat w` times :math:`R` – anything else would
   be inconsistent with the constraints of orthogonality and determinant
   1.

   Note also that, assuming :math:`R(0)=I`, the solution to the
   differential equation :math:`\dot R(t) = \hat w~ R(t)` can be written
   as :math:`R(t)=\exp(t \hat w)`, where here the exponential function
   notation is used to denote a more general so-called exponential map,
   as used in the context of Lie groups. It also follows that
   :math:`R(w)` from (`[eqRodriguez] <#eqRodriguez>`__) is the rotation
   matrix you get when you rotate for 1 second with angular velocity
   described by :math:`w`.

Quaternion
   (I’m not describing the general definition, only the “quaternion to
   represent rotation” definition.) A quaternion is a unit length 4D
   vector :math:`r\in{\mathbb{R}}^4`; the first entry :math:`r_0` is
   related to the rotation angle :math:`\theta` via
   :math:`r_0=\cos(\theta/2)`, the last three entries
   :math:`\bar r\equiv r_{1:3}` are related to the unit length rotation
   axis :math:`\underline w` via
   :math:`\bar r = \sin(\theta/2)~ \underline w`.

   -  The inverse of a quaternion is given by negating :math:`\bar r`,
      :math:`r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} =
      (r_0,-\bar r)` (or, alternatively, negating :math:`r_0`).

   -  The concatenation of two rotations :math:`r`, :math:`r'` is given
      as the quaternion product

      .. math::

         \begin{aligned}
         \label{eqQuat}
         r \circ r'
          = (r_0 r'_0 - \bar r^{\!\top\!}\bar r',~
             r_0 \bar r' + r'_0 \bar r + \bar r' \times \bar r)\end{aligned}

   -  The application of a rotation quaternion :math:`r` on a vector
      :math:`x` can be expressed by converting the vector first to the
      quaternion :math:`(0,x)`, then computing

      .. math::

         \begin{aligned}
         r \cdot x = (r \circ (0,x) \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1})_{1:3} ~,\end{aligned}

      I think a bit more efficient is to first convert the rotation
      quaternion :math:`r` to the equivalent rotation matrix :math:`R`:

   -  Conversion to/from a matrix: A quaternion rotation :math:`r`
      convertes to the rotation matrix

      .. math::

         \begin{aligned}
         R
          &=  \left(\begin{array}{ccc}
             1-r_{22}-r_{33} & r_{12}-r_{03} &    r_{13}+r_{02} \\
             r_{12}+r_{03} &   1-r_{11}-r_{33} &  r_{23}-r_{01} \\
             r_{13}-r_{02} &   r_{23}+r_{01} &    1-r_{11}-r_{22}
             \end{array}\right)  \\ & ~ r_{ij} := 2 r_i r_j ~.\end{aligned}

      (Note: In comparison to (`[eqRodriguez] <#eqRodriguez>`__) this
      does not require to compute a :math:`\sin` or :math:`\cos`.)
      Inversely, the quaternion :math:`r` for a given matrix :math:`R`
      is

      .. math::

         \begin{aligned}
             r_0 &= {\frac{1}{2}}\sqrt{1+{\rm tr}R}\\
             r_3 &= (R_{21}-R_{12})/(4 r_0)\\
             r_2 &= (R_{13}-R_{31})/(4 r_0)\\
             r_1 &= (R_{32}-R_{23})/(4 r_0) ~.\end{aligned}

Angular velocity :math:`\to` quaternion velocity
   Given an angular velocity :math:`w\in{\mathbb{R}}^3` and a current
   quaternion :math:`r(t)\in{\mathbb{R}}^4`, what is the time derivative
   :math:`\dot r(t)` (in analogy to Eq. (\ `[eqDotR] <#eqDotR>`__))? For
   simplicity, let’s first assume :math:`|w|=1`. For a small time
   interval :math:`\delta`, :math:`w` generates a rotation vector
   :math:`\delta w`, which converts to a quaternion

   .. math::

      \begin{aligned}
      \Delta r = (\cos(\delta/2), \sin(\delta/2) w) ~.\end{aligned}

   That rotation is concatenated LHS to the original quaternion,

   .. math::

      \begin{aligned}
      r(t+\delta)
       = \Delta r \circ r(t) ~.\end{aligned}

   Now, if we take the derivative w.r.t. \ :math:`\delta` and evaluate
   it at :math:`\delta=0`, all the :math:`\cos(\delta/2)` terms become
   :math:`-\sin(\delta/2)` and evaluate to zero, all the
   :math:`\sin(\delta/2)` terms become :math:`\cos(\delta/2)` and
   evaluate to one, and we have

   .. math::

      \begin{aligned}
      \label{eqQuatVel}
      \dot r(t)
      &= {\frac{1}{2}}( - w^{\!\top\!}\bar r,~  r_0 w + \bar r \times w )
       = {\frac{1}{2}}(0,w) \circ r(t)\end{aligned}

   Here :math:`(0,w)\in{\mathbb{R}}^4` is a four-vector; for
   :math:`|w|=1` it is a normalized quaternion. However, due to the
   linearity the equation holds for any :math:`w`.

Quaternion velocity :math:`\to` angular velocity
   The following is relevant when taking the derivative
   w.r.t. quaternion parameters, e.g., of a ball joint represented as
   quaternion. Given :math:`\dot r`, we have

   .. math::

      \begin{aligned}
      \label{eq37}
      \dot r \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}
      &= {\frac{1}{2}}(0,w) \circ r \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} = {\frac{1}{2}}(0,w) ~,\quad w = 2~ [\dot r \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}]_{1:3}\end{aligned}

   which allows us to read off the angular velocity :math:`w` induced by
   a change of quaternion :math:`\dot r`. However, the RHS zero will
   hold true only iff :math:`\dot
   r` is orthogonal to :math:`r` (where
   :math:`\dot r^{\!\top\!}r = \dot r_0 r_0 + \dot{\bar
   r}{}^{\!\top\!}\bar r = 0`, see ). In case
   :math:`\dot r^{\!\top\!}r \not=0`, the change in length of the
   quaternion does not represent any angular velocity; in typical
   kinematics engines a non-unit length is ignored. Therefore one first
   orthogonalizes :math:`\dot
   r \gets \dot r - r(\dot r^{\!\top\!}r)`.

   As a special case of application, consider computing the partial
   derivative w.r.t. quaternion parameters, where :math:`\dot r` is the
   4D unit vectors :math:`e_0,..,e_3`. In this case, the
   orthogonalization becomes simply :math:`\dot r \gets e_i - r r_i` and
   (`[eq37] <#eq37>`__) becomes

   .. math::

      \begin{aligned}
      (e_i - r_i r) \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}
        &= e_i \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} - r_i (1,0,0,0) ~,\quad
        w_i
       = 2~ [e_i \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}]_{1:3} ~,\end{aligned}

   where :math:`w_i` is the rotation vector implied by
   :math:`\dot r = e_i`. In case the original quaternion :math:`r`
   wasn’t normalized (which could be, if a standard optimization
   algorithm searches in the quaternion parameter space), then :math:`r`
   actually represents the normalized quaternion
   :math:`\bar r = \frac{1}{\sqrt{r^2}} r`, and (due to linearity of the
   above), the rotation vector implied by :math:`\dot r = e_i` is

   .. math::

      \begin{aligned}
      \label{eqQuatJac}
      w_i
      &= \frac{2}{\sqrt{r^2}}~ [e_i \circ r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}]_{1:3} ~.\end{aligned}

   This defines a :math:`3\times 4` **quaternion Jacobian**
   :math:`J_{:i} = w_i` with 4 columns :math:`w_i`, so that
   :math:`w = J \dot r` is the angular velocity induced by a quaternion
   velocity :math:`\dot r` (accounting for all implicit normalizations).

.. _secTransformations:

Transformations
~~~~~~~~~~~~~~~

We can represent a transformation as:

A homogeneous matrix
   is a :math:`4\times 4`-matrix of the form

   .. math::

      \begin{aligned}
      T =  \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) \end{aligned}

   where :math:`R` is a :math:`3\times 3`-matrix (rotation in our case)
   and :math:`t` a :math:`3`-vector (translation).

   In homogeneous coordinates, vectors :math:`x\in{\mathbb{R}}^3` are
   expanded to 4D vectors
   :math:`\left(\begin{array}{c}x\\1\end{array}\right)  \in {\mathbb{R}}^4`
   by appending a 1.

   Application of a transform :math:`T` on a vector
   :math:`x\in{\mathbb{R}}^3` is then given as the normal matrix-vector
   product

   .. math::

      \begin{aligned}
      x' = T \cdot x
       &= T~  \left(\begin{array}{c}x \\ 1\end{array}\right) 
        =  \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) ~  \left(\begin{array}{c}x \\ 1\end{array}\right) 
        =  \left(\begin{array}{c}Rx + t \\ 1\end{array}\right)  ~.\end{aligned}

   Concatenation is given by the ordinary 4-dim matrix-matrix product.

   The inverse transform is

   .. math::

      \begin{aligned}
      T^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}
       &=  \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) ^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}
        =  \left(\begin{array}{cc}R^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} & -R^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} t \\ 0 & 1\end{array}\right) \end{aligned}

Translation and quaternion:
   A transformation can efficiently be stored as a pair :math:`(t,r)` of
   a translation vector :math:`t` and a rotation quaternion :math:`r`.
   Analogous to the above, the application of :math:`(t,r)` on a vector
   :math:`x` is :math:`x' = t + r\cdot x`; the inverse is
   :math:`(t,r)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} = (-r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}\cdot t, r^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1})`;
   the concatenation is
   :math:`(t_1,r_1) \circ (t_2,r_2) = (t_1 + r_1\cdot t_2, r_1 \circ r_2)`.

Sequences of transformations
   by :math:`T_{A\to
   B}` we denote the transformation from frame :math:`A` to frame
   :math:`B`. The frames :math:`A` and :math:`B` can be thought of
   coordinate frames (tuples of an offset (in an affine space) and three
   local orthonormal basis vectors) attached to two bodies :math:`A` and
   :math:`B`. It holds

   .. math::

      \begin{aligned}
      T_{A\to C} = T_{A\to B} \circ T_{B\to C}\end{aligned}

   where :math:`\circ` is the concatenation described above. Let
   :math:`p` be a point (rigorously, in the affine space). We write
   :math:`p^A` for the coordinates of point :math:`p` relative to frame
   :math:`A`; and :math:`p^B` for the coordinates of point :math:`p`
   relative to frame :math:`B`. It holds

   .. math::

      \begin{aligned}
      p^A = T_{A\to B}~ p^B ~.\end{aligned}

.. _secTransNotation:

A note on “forward” vs. “backward” of frame and coordinate transforms
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instead of the notation :math:`T_{A\to B}`, other text books often use
notations such as :math:`T_{AB}` or :math:`T^A_B`. A common question
regarding notation :math:`T_{A\to B}` is the following:

   *The notation :math:`T_{A\to B}` is confusing, since it transforms
   coordinates from frame :math:`B` to frame :math:`A`. Why is the
   notation not the other way around?*

I think the notation :math:`T_{A\to B}` is intuitive for the following
reasons. The core is to understand that a transformation can be thought
of in two ways: as a transformation of the *coordinate frame itself*,
and as transformation of the *coordinates relative to a coordinate
frame*. I’ll first give a non-formal explanation and later more formal
definitions of affine frames and their transformation.

Think of :math:`T_{W\to B}` as translating and rotating a real rigid
body: First, the body is located at the world origin; then the body is
moved by a translation :math:`t`; then the body is rotated (around its
own center) as described by :math:`R`. In that sense,
:math:`T_{W\to B} =  \left(\begin{array}{cc}R & t \\ 0
& 1\end{array}\right)` describes the “forward” transformation of the
body. Consider that a coordinate frame :math:`B` is attached to the
rigid body and a frame :math:`W` to the world origin. Given a point
:math:`p` in the world, we can express its coordinates relative to the
world, :math:`p^W`, or relative to the body :math:`p^B`. You can
convince yourself with simple examples that
:math:`p^W = T_{W\to B}~ p^B`, that is, :math:`T_{W\to B}` *also*
describes the “backward” transformation of body-relative-coordinates to
world-relative-coordinates.

Formally: Let :math:`(A,V)` be an affine space. A coordinate frame is a
tuple :math:`(o,\boldsymbol e_1,..,\boldsymbol e_n)` of an origin
:math:`o \in A` and basis vectors :math:`\boldsymbol e_i \in V`. Given a
point :math:`p\in A`, its coordinates :math:`p_{1:n}` w.r.t. a
coordinate frame :math:`(o,\boldsymbol e_1,..,\boldsymbol e_n)` are
given implicitly via

.. math::

   \begin{aligned}
   p = o + \sum\nolimits_i p_i \boldsymbol e_i ~.\end{aligned}

A transformation :math:`T_{W\to B}` is a (“forward”) transformation of
the coordinate frame itself:

.. math::

   \begin{aligned}
   (o^B,\boldsymbol e^B_1,..,\boldsymbol e^B_n)
    &= (o^W + t, R\boldsymbol e^W_1,..,R\boldsymbol e^W_n)\end{aligned}

where :math:`t\in V` is the affine translation in :math:`A` and
:math:`R` the rotation in :math:`V`. Note that the coordinates
:math:`(\boldsymbol e^B_i)^W_{1:n}` of a basis vector
:math:`\boldsymbol e^B_i` relative to frame :math:`W` are the columns of
:math:`R`:

.. math::

   \begin{aligned}
   \boldsymbol e^B_i
    &= \sum_j (\boldsymbol e^B_i)^W_j \boldsymbol e^W_j
     = \sum_j R_{ji} \boldsymbol e^W_j\end{aligned}

Given this transformation of the coordinate frame itself, the
coordinates transform as follows:

.. math::

   \begin{aligned}
   p &= o^W + \sum_i p^W_i~ \boldsymbol e^W_i \\
   p &= o^B + \sum_i p^B_i~ \boldsymbol e^B_i \\
     &= o^W + t + \sum_i p^B_i~ (R \boldsymbol e^W_i) \\
     &= o^W + \sum_i t^W_i~ e^W_i + \sum_j p^B_j~ (R \boldsymbol e^W_j) \\
     &= o^W + \sum_i t^W_i~ e^W_i + \sum_j p^B_j~ (\sum_i R_{ij}~ \boldsymbol e^W_i) \\
     &= o^W + \sum_i \Big[t^W_i + \sum_j R_{ij}~ p^B_j\Big]~ e^W_i \\
   \Rightarrow
    &~ p^W_i = t^W_i + \sum_j R_{ij}~ p^B_j ~.\end{aligned}

Another way to express this formally: :math:`T_{W\to B}` maps
*covariant* vectors (including “basis vectors”) forward, but
*contra-variant* vectors (including “coordinates”) backward.

Splines
-------

A spline is a piece-wise polynomial path
:math:`x:[0,T] \maps {\mathbb{R}}^n`.

Let’s first clearly distinguish the use of words **knot**, **waypoint**,
and **control point**:

-  A knot :math:`t_i` is a point in *time*, :math:`t_i \in {\mathbb{R}}`
   (usually :math:`t_i \in [0,T]`). The path is polynomial between
   knots. We have a non-decreasing sequence of knots :math:`t_0,..,t_m`
   (usually with :math:`t_0=0` and :math:`t_m=T`) which partition the
   time interval :math:`[0,T]` into pieces
   :math:`[t_i, t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}]` so
   that the path is just polynomial in each piece. Note that we may
   often have double or triple knots, meaning that several consecutive
   knots
   :math:`t_i = t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}` are
   equal, especially at the beginning and end.

-  A waypoint :math:`x_i` is a point on the path, typically at a knot,
   :math:`x_i = x(t_i)`. So a path really passes through a waypoint. At
   waypoints, we often also care about velocities :math:`v_i` (or
   accelerations), where :math:`v_i = \dot x(t_i)`.

-  A control point :math:`z_j` is (usually) not a point *on* the path,
   but it indirectly defines the path as an linear combination of
   several control points. B-splines, defined below, make this explicit.

In robotics, there are two main conventions to define and parameterize
splines: Hermite splines and B-splines. Hermite splines are defined by
the knot sequence and explicitly prescribing waypoints :math:`x_i` and
(for cubic) velocities :math:`v_i` at each knot (for quintic also
acceperations :math:`a_i`). In contrast, B-splines are specified by the
knot sequence and :math:`K` control points :math:`z_j`. As in B-splines
we do not need to provide velocities as part of the specification, they
are sometimes easier to use in practical robotics. However, the
resulting path does not go (exactly) through the provided control points
– the actual waypoints are implicit and ensuring exact prescribed
waypoints implies solving a subproblem.

Cubic splines are a common choice in robotics, as they have a still
continuous (piece-wise linear) acceleration profile, and therefore
limited jerk (3rd time derivative).

Let’s start with Cubic Hermite splines.

Code References
---------------

.. _secNLP:

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
:math:`b_l,b_u\in{\mathbb{R}}^n`. However, we want to explicitly account
for **least squares** costs (sum-of-squares), so that we extend the form
to

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ f(x) + r(x)^{\!\top\!}r(x) ~~~\text{s.t.}~~~ g(x)\le 0,~ h(x) = 0  ~,\end{aligned}

with :math:`r:~ {\mathbb{R}}^n \to {\mathbb{R}}^{d_r}`. In technical
terms, the solver needs to be provided with:

-  the problem “signature”: dimension :math:`n`, dimensions
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

-  the problem “signature”: dimension :math:`n`, feature types
   :math:`\rho`, bounds :math:`b_l, b_u \in {\mathbb{R}}^n`,

-  a single differentiable **feature** function
   :math:`\phi: X \to {\mathbb{R}}^K`, with Jacobian function
   :math:`J = \partial_x \phi(x)`,

-  optionally a Hessian function for the sum of all ``f``-terms,

-  and typically also an initialization sampler :math:`x_0 \sim p(x)`,
   that provides starting :math:`x_0`.

In the rai code, an NLP is therefore declared as

::

     //signature
     uint dimension;  ObjectiveTypeA featureTypes;  arr bounds_lo, bounds_up;

     //essential method
     virtual void evaluate(arr& phi, arr& J, const arr& x);

     //optional
     virtual arr  getInitializationSample(const arr& previousOptima={});
     virtual void getFHessian(arr& H, const arr& x);

.. _secYamlGraph:

Yaml-Graph Files
~~~~~~~~~~~~~~~~

We use yaml-style files throughout. These are the file representation of
internal data structures such as dictionaries (anytype key-value maps)
used for parameter files or other structure data, but esp. also graphs.
The (semantic) extensions relative to yaml are:

-  An @Include@ node allows to hierarchically include files. This means
   that while each local file can be parsed with a standard yaml parser,
   an outer loop has to check for @Include@ nodes and coordinate loading
   sub-files.

-  As an implication of the above, we allow for a special @path@ type,
   as URLs embraced by ``<...>``. This becomes necessary as file values
   need to be interpreted relative to the path of the loading file. So
   when such a file is parsed we not only store the filename string, but
   also the path of the loading file to ensure we know its absolute
   path.

-  We also allow @Edit@ and @Delete@ tags, which allow us to
   edit/overwrite the value of previously defined nodes, as well as
   delete previously defined nodes.

-  Finally, the name of a node can include a list of parents: E.g. @A (B
   C): shape: box@ denotes a node with key @A@ that is a child of @B@
   and @C@. The semantics of this is that @A@ is a (directed) edge
   between B and C. This is analogous to a dot declaration @B -> C [
   shape=box ]@.

-  Note that all of the above is still yaml syntax, the outer parser
   only adds additional interpretation (post-processing) of @Include,
   Edit, Delete@ tags, @<..>@ values, and @(..)@ in names.

Within rai, .g-files are used to represent parameter files, robotic
configurations (:math:`\sim` URDF), 1st order logic, factor graphs,
optimization problems. The underlying data structure is used, e.g., as
any-type container, Graph, or auto-convertion to python dictionaries.

Subgraphs may contain nodes that have parents from the containing graph,
or from other subgraphs of the containing graph. Some methods of the
``Graph`` class (to find nodes by key or type) allow to specify whether
also nodes in subgraphs or parentgraphs are to be searched. This
connectivity across (sub)-graphs e.g. allows to represent logic
knowledge bases.

The shows this file syntax is used to specify robot/environment
configurations.

Cameras
-------

Image, Camera, & World Coordinates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this section, we use the following notation for coordinates of a 3D
point:

-  world coordinates :math:`X`,

-  camera coordinates :math:`x` (so that :math:`X = T x`, where
   :math:`T\equiv T_{W\to C}` is the camera position/orientation, also
   called **extrinsic parameter**),

-  image coordinates :math:`u=(u_x,u_y,u_d)`, with the pixel coordinates
   :math:`(u_x,u_y)` and depth coordinate :math:`u_d`, details as
   followed.

The pixel coordinates :math:`(u_x,u_y)` indicate where a point appears
on the image plane. The :math:`x`-axis always points to the right, but
there are two conventions for the :math:`y`-axis:

-  :math:`y`-up: The :math:`y`-axis points upward. This is consistent to
   how a diagram is typically drawn on paper: :math:`x`-axis right,
   :math:`y`-axis up. However, a consequence is that the :math:`z`-axis
   then points backward, i.e., pixels in front of the camera have
   negative depth :math:`u_d`.

-  :math:`y`-down: The :math:`y`-axis points down. This is consistent to
   how pixels are typically indexed in image data: counting rows from
   top to bottom. So when defining pixel coordinates :math:`(u_x,u_y)`
   literally to be pixel indices in image data, :math:`y`-down is the
   natural convention. A consequence is that the :math:`z`-axis points
   forward, i.e., pixels in front of the camera have a positive depth
   :math:`u_d`, which might also be more intuitive.

The transformation from camera coordinates :math:`x` to image
coordinates :math:`u` is involves perspective projection. For better
readability, let’s write (only in this equation)
:math:`x \equiv (\texttt{x},\texttt{y},\texttt{z})`. Then the mapping is

.. math::

   \begin{aligned}
   \label{eqxtou}
   u =  \left(\begin{array}{c}u_x \\ u_y \\ u_d\end{array}\right) 
   &=  \left(\begin{array}{c}(f_x \texttt{x}+ s \texttt{y})/\texttt{z}+ c_x\\ f_y \texttt{y}/\texttt{z}+ c_y \\ \texttt{z}\end{array}\right)  ~.\end{aligned}

Here, the five so-called **intrinsic parameters**
:math:`f_x,f_y,c_x,c_y,s` are the focal length :math:`f_x,f_y`, the
image center :math:`c_x,c_y`, and a image skew :math:`s` (which is
usually zero). E.g., for an image of height :math:`H` and width
:math:`W`, and vertical full view angle :math:`\alpha`, we typically
have an image center :math:`c_x \approx H/2, c_y \approx W/2` and a
focal length :math:`f_y
= \frac{H}{2 \tan(\alpha/2)}`, e.g., for :math:`\alpha=90^\circ`,
:math:`f_y = H/2`. For a typical camera :math:`f_x \approx f_y`.

Inversely, if we have image coordinates :math:`u` and want to convert to
cartesian camera coordinates, we have (assuming :math:`s=0`)

.. math::

   \begin{aligned}
   x
   &=  \left(\begin{array}{c}(u_x - c_x) u_z / f_x\\ (u_y - c_y) u_z / f_y \\ u_z\end{array}\right)  ~.\end{aligned}

Homogeneous coordinates & Camera Matrix :math:`P`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First a brief definition: *A homogeneous coordinate
:math:`\boldsymbol x=(x_1,..,x_n,w)` is a (redundant) description of the
:math:`n`-dim point*

.. math:: {\cal P}(\boldsymbol x)=  \left(\begin{array}{c}x_1/w \\ \vdots \\ x_n/w\end{array}\right)  ~.

Note that two coordinates :math:`(x_1,..,x_n,w)` and
:math:`(\lambda x_1,..,\lambda x_n,\lambda w)` are “equivalent” in that
they describe the same point. The operation :math:`{\cal P}` is
*non-linear* and called **perspective projection**. In this section, we
write homogeneous coordinates in bold :math:`\boldsymbol x`.

Back to our camera setting: Let :math:`\boldsymbol x` and
:math:`\boldsymbol X` be homogeneous camera and world coordinates of a
point (typically both have :math:`w=1` as last entry). Then the pose
transformation :math:`T` can be written as :math:`4\times` matrix such
that

.. math:: \boldsymbol x = T^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} \boldsymbol X ~.

Given camera coordinates :math:`x = ``(x,y,z)''`, we can write
(`[eqxtou] <#eqxtou>`__)

.. math::

   \begin{aligned}
   \boldsymbol u
   &= K x
   =  \left(\begin{array}{c}f_x x + s y + c_x z\\ f_y y + c_y z \\ z\end{array}\right)  ~,\quad
   K =  \left(\begin{array}{ccc}f_x & s & c_x \\ & f_y & c_y \\ & & 1 \end{array}\right)  ~,\quad
   {\cal P}(\boldsymbol u)
   =  \left(\begin{array}{c}  (f_x x + s y)/z + c_x\\ f_y y/z + c_y \end{array}\right)  ~,\end{aligned}

where :math:`\boldsymbol u` are homogeneous *pixel* coordinates, and
:math:`{\cal P}(\boldsymbol u)` the actual pixel coordinates, which
would have to be augmented with :math:`z` again to get the :math:`u`
including depth coordinate.

The :math:`3\times 3` matrix :math:`K` includes the 5 general intrinsic
parameters. Writing the inverse transformation
:math:`T^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}` as a
:math:`3\times 4` matrix
:math:`\left(\begin{array}{cc}R^{\!\top\!}& -R^{\!\top\!}t\end{array}\right)`
with rotation :math:`R` and translation :math:`t`, we can write the
relation between :math:`\boldsymbol u` and homogeneous world coordinates
:math:`\boldsymbol X` as

.. math::

   \begin{aligned}
   \boldsymbol u = P \boldsymbol X
   ~,\quad\text{with~} P =  \left(\begin{array}{cc}K & 0\end{array}\right) ~ T^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} =  \left(\begin{array}{cc}K & 0\end{array}\right) ~  \left(\begin{array}{cc}R^{\!\top\!}& -R^{\!\top\!}t \\ & 1\end{array}\right)  =  \left(\begin{array}{cc}KR^{\!\top\!}& -KR^{\!\top\!}t\end{array}\right)  ~,\end{aligned}

where :math:`P` is the :math:`3\times 4` **camera matrix**, which
subsumes 5 intrinsic and 6 extrinsic (3 rotation, 3 translation)
parameters. Except for absolute scaling (the 1 in the definition of
:math:`K`), this fully parameterizes a general affine transform.

Calibration as Estimating :math:`P,K,R,t` from Depth Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Assuming we have data of pairs :math:`(\boldsymbol u, \boldsymbol X)`,
we can use the basic equation :math:`\boldsymbol u = P \boldsymbol X` to
retrieve :math:`P` in closed from, and in a second step retrieve the
intrinsic and extrinsic camera parameters from :math:`P`. Note that here
we discuss the situation where we have the “right” :math:`\boldsymbol u`
in the data – and not only the pixel coordinates
:math:`{\cal P}(\boldsymbol u)`! This means that we assume we have data
entries :math:`\boldsymbol u = (u_x u_d, u_y u_d, u_d)` which includes
the true depth :math:`u_d`. So this method is only applicable when we
want to calibrate a depth camera.

Given data :math:`D = \{(\boldsymbol u_i, \boldsymbol X_i)\}_{i=1}^n`,
we want to minimize the squared error

.. math::

   \begin{aligned}
   \argmin_P \sum_i (\boldsymbol u_i - P \boldsymbol X_i)^2 = [U - P X]^2 ~,\end{aligned}

where :math:`U` and :math:`X` are the stacked :math:`\boldsymbol u_i`
and :math:`\boldsymbol X_i`, respectively. The solution is
:math:`P = U^{\!\top\!}X (X^{\!\top\!}X)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}`.
Comparing with the form of :math:`P` above, we can decompose it and
extract explicit :math:`K, R, t` using

.. math::

   \begin{aligned}
     (K,R^{\!\top\!}) &\gets \text{RQ-decomposition}(P_{1:3,:}) \\
     t &\gets -(K R^{\!\top\!})^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} P_{4,:}\end{aligned}

However, when defining
:math:`\bar u = (\boldsymbol u,1) = (u_x u_z, u_y u_z, u_z, 1)` (with
additional 1 agumented), we can also write the inverse linear relation
to the non-homogeneous world coordinate :math:`X`:

.. math::

   \begin{aligned}
   X  = P^+ \bar u~,\quad\text{with~} P^+ =  \left(\begin{array}{cc}R K^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} & t\end{array}\right)  \bar u~,\end{aligned}

Using data :math:`X` (:math:`3\times n`) and :math:`U`
(:math:`4\times n`) the optimum is
:math:`P^+ = X^{\!\top\!}U (U^{\!\top\!}U)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}`.
We can decompose it using

.. math::

   \begin{aligned}
     t &\gets P^+_{3,:} \\
     (K,R^{\!\top\!}) &\gets \text{RQ-decomposition}( [P^+_{1:3,:}]^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} ]\end{aligned}

