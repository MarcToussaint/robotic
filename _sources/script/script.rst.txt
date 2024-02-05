Lecture Script
==============

Introduction
------------

Reference material
~~~~~~~~~~~~~~~~~~

In terms of background, please refer to the Maths for Intelligent
Systems
<https://www.user.tu-berlin.de/mtoussai/teaching/Lecture-Maths.pdf> as
well as the Intro to Robotics
<https://www.user.tu-berlin.de/mtoussai/teaching/Lecture-Robotics.pdf>
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
<https://marctoussaint.github.io/robotics-course/> for setting up the
python package. This includes a series of tutorials, which can also be
downloaded here <https://github.com/MarcToussaint/rai-tutorials>.

Scene & Robot Description
-------------------------

Generally speaking, a scene is a collection of objects (including robot
parts). We typically assume objects to be rigid bodies with fixed shape
– which clearly is a simplification relative to real world. More about
this below, in section `1.16.1 <#secShapes>`__.

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
a rotation – see Appendix `1.12 <#appTransforms>`__ for more details.
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
`1.12 <#appTransforms>`__ introduces to all these representations and
derives conversion equations to relate them.

The 1st tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_1_intro.html>
illustrates how you can manually define frames in a configuration and
set absolute or relative transformations.

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
   X_3 &= Q_{W\to 1} \circ Q_{1\to2} \circ Q_{2\to3} ~.\end{aligned}

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
   x^W &= Q_{W\to 1}~ Q_{1\to2}~ Q_{2\to3}~ x^3 = X_3~ x^3 ~.\end{aligned}

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
*contra-variant*. The appendix `1.12.2.1 <#secTransNotation>`__ explains
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

The 1st tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_1_intro.html>
also demonstrates how to define a frame a *child* of another, thereby
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
   0 & \cos(q_i) & -\sin(q_i) & 0 \\
   0 &  \sin(q_i) & \cos(q_i) & 0 \\
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

In the 1st tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_1_intro.html>,
when a joint is defined for the first time, play around with alternative
joint types, e.g. a ``quatBall``. The tutorial also lists which joint
types are pre-defined.

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

In the tutorial on configuration editing
<https://marctoussaint.github.io/robotic/tutorials/config_3_import_edit.html>
you find a section on interactively editing a scene description file
``mini.g``. Using this you can try to invent your own robot and
environment. The tutorial also shows how to load pre-defined robot
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
configuration (see `1.16.2 <#secKinematics>`__).

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

The 1st tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_1_intro.html>
illustrates how you get the joint vector :math:`q` and set it. This way
you can animate the configuration. Also the positions and orientations
of all frames can be queried directly – realizing the most basic kind of
forward kinematics.

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
:math:`i` also contributes a column to the basic matrix
:math:`J^{\textsf{pos}}(p)`, but contributes notion to
:math:`J^{\textsf{ang}}` (as it does not imply rotational velocity in
the sub-branch). Specifically, let :math:`a_j` be the translational axis
of the joint with dof index :math:`j`, then it simply contributes a
column

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
matrix :math:`J(r)\in{\mathbb{R}}^{3 \times 4}` describes how a
variation of a quaternion :math:`r` induces a 3D rotation vector
relative to the *output* space of :math:`r`. I call this the quaternion
Jacobian. The derivation is found in the appendix when discussion how a
quaternion velocity implies and angular velocity. The multiplication
with :math:`R_j` transforms this rotation vector to world coordinates.
The division by :math:`|q_j|` accounts when the dof :math:`q_j` is not
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

The features tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_2_features.html>
introduces to features that are readily implemented in the rai code.

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
finite difference of a zero-order feature. In general, a higher-order
feature is a differentiable mapping

.. math:: \phi: (q_0,q_1,..,q_k) \mapsto \mathbb{R}^D

of a :math:`(k+1)`-tuple of configurations to a :math:`D`-dimensional
space. This is typically used in the context of **path configurations**,
which is a sequence of configurations used in path optimization.

Given any 0-order feature :math:`\phi`, by default that defines its 1st
and 2nd order feature as

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
   q^* = \text{argmin}_q f(q) ~~\text{s.t.}~~\phi(q) = y^* ~.\end{aligned}

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
`1.14.1 <#secNLP>`__ provides a technical reference of how we define
NLPs mathematically and in code. Essentially, an NLP is specified by
*adding objectives*, where each objective is given by a feature function
:math:`\phi_i` and an indicator :math:`\varrho_i` that defines whether
the feature contributes a linear cost (``f``), sum-of-squares cost
(``sos``), equality constraint (``eq``), or inequality constraint
(``ineq``) to the NLP.

The 1st KOMO tutorial
<https://marctoussaint.github.io/robotic/tutorials/komo_1_intro.html>
illustrates how an Inverse Kinematics problem can be specified as NLP.
The core is the ``addObjective`` method, which adds a kinematic feature
(optimally with transformed by scaling and target) as a cost or
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
   &= - [{\nabla_{\!\!f}^2}(q)]^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} {\nabla_{\!\!f}}(q) = - (J^{\!\top\!}C J + W)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} [J^{\!\top\!}C (\phi(q)-y^*) + W (q-q_0) ]\end{aligned}

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
:math:`x:[0,T] \to {\mathbb{R}}^n`. Let’s first clearly distinguish the
use of words *knot*, *waypoint*, and *control point*:

-  A **knot** :math:`t_i` is a point in *time*,
   :math:`t_i \in {\mathbb{R}}`, we assume :math:`t_i \in [0,T]`. For a
   spline, we have a non-decreasing sequence of knots :math:`t_0,..,t_m`
   (we assume :math:`t_0=0` and :math:`t_m=T`) which partition the time
   interval :math:`[0,T]` into pieces
   :math:`[t_i, t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}]` so
   that the path is polynomial in each piece. Note that we may often
   have double or triple knots, meaning that several consecutive knots
   :math:`t_i = t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}` are
   equal, especially at the beginning and end.

-  A **waypoint** :math:`x_i` is a point on the path, typically at a
   knot, :math:`x_i = x(t_i)`. So a path really passes through a
   waypoint. At waypoints, we often also care about velocities
   :math:`v_i` and accelerations :math:`\alpha_i`, where
   :math:`v_i = \dot x(t_i)`, :math:`a_i = \ddot x(t_i)`, .

-  A **control point** :math:`z_j` is (usually) not a point *on* the
   path, but it indirectly defines the path as an linear combination of
   several control points. B-splines, defined below, make this explicit.

In robotics, there are two main conventions to define and parameterize
splines: Hermite splines and B-splines. Hermite splines are defined by
the knot sequence and explicitly prescribing waypoints :math:`x_i` and
(for cubic) velocities :math:`v_i` at each knot (for quintic also
acceperations :math:`a_i`). In contrast, B-splines are specified by the
knot sequence and :math:`K` control points :math:`z_j`. As in B-splines
we do not need to provide velocities as part of the specification, they
are sometimes easier to use in practice. However, the resulting path
does not go (exactly) through the provided control points – the actual
waypoints are implicit and ensuring exact prescribed waypoints implies
solving a subproblem.

Cubic splines are a common choice in robotics, as they have a still
continuous (piece-wise linear) acceleration profile, and therefore
limited jerk (3rd time derivative).

In the following we first discuss a single cubic spline-piece as a means
of control, then Hermite splines, then B-splines.

Single cubic spline for timing-optimal control to a target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following discusses a single cubic spline piece and and how to use
it for timing-optimal control to a target. Although very simple, the
method is a powerful alternative to typical PD-control to a target. It
also lays foundations on how timing-optimality can be realized with
Hermite splines.

Consider a cubic polynomial :math:`x(t) = a t^3 + b t^2 + c t + d`.
Given four boundary conditions
:math:`x(0)=x_0, \dot x(0) = v_0, x(\theta) = x_\theta, \dot x(\theta) = v_\theta`,
the four coefficients are

.. math::

   \begin{aligned}
   d &= x_0 ~, \\
   c &= \dot x_0 ~, \\
   b &= \frac{1}{\theta^2}\Big[ 3(x_\theta-x_0) - \theta(\dot x_\theta+ 2 \dot x_0) \Big] ~, \\
   a &= \frac{1}{\theta^3}\Big[ - 2(x_0-x_\theta) + \theta(\dot x_\theta+ \dot x_0) \Big] ~.\end{aligned}

This cubic spline is in fact the solution to an optimization problem,
namely it is the path that minimizes accelerations between these
boundary conditions and it can therefore be viewed as the solution to
optimal control with acceleration costs:

.. math::

   \begin{aligned}
   \min_x~ \int_0^\tau \ddot x(t)^2~ dt 
   \quad~~\text{s.t.}~~ \left(\begin{array}{c}x(0)\\\dot x(0)\end{array}\right) = \left(\begin{array}{c}x_0\\v_0\end{array}\right) ,~
    \left(\begin{array}{c}x(\tau)\\\dot x(\tau)\end{array}\right) = \left(\begin{array}{c}x_1\\v_1\end{array}\right)  ~.\end{aligned}

The minimal costs can analytically be given as

.. math::

   \begin{aligned}
   \int_0^T \ddot x(t)^2 dt
   %% &= \int_0^T (6 a t + 2 b)^2 ~ dt \\
   %% &= \int_0^T (36 a^2 t^2 + 4 b^2 + 24 abt) ~ dt \\
   %% &= 4 b^2 [t]_0^T + 24 ab [\half t^2]_0^T + 36 a^2 [\frac{1}{3} t^3]_0^T \\
   &= 4 \tau b^2  + 12 \tau^2 ab + 12 \tau^3 a^2 \\
   &= \frac{12}{\tau^3}~[(x_1 - x_0)-\frac{\tau}{2}(v_0+v_1)]^2+\frac{1}{\tau}(v_1-v_0)^2 \label{eqLeap}\\
   &= \frac{12}{\tau^3} D^{\!\top\!}D + \frac{1}{\tau} V^{\!\top\!}V ~,\quad D := (x_1 - x_0)-\frac{\tau}{2}(v_0+v_1),~ V:=v_1-v_0,~ \\
   &= \tilde D^{\!\top\!}\tilde D + \tilde V^{\!\top\!}\tilde V ~,\quad
   \tilde D := \sqrt{12}~ \tau^{-\frac{3}{2}}~ D,~ \tilde V := \tau^{-{\frac{1}{2}}}~ V ~,
   \label{eqLeapSOS}\end{aligned}

where we used some help of computer algebra to get this right.

Eq. (\ `[eqLeap] <#eqLeap>`__) explicitly gives the optimal cost in
terms of boundary conditions :math:`(x_0,v_0,x_1,v_1)` and time
:math:`\tau`. This is a very powerful means to optimize boundary
conditions and :math:`\tau`. The following is a simple application that
realizes reactive control.

Single-piece optimal timing control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Consider the system is in state :math:`(x,\dot x)` and you want to
control it to a reference point
:math:`(x_\text{ref}, \dot x_\text{ref}=0)`. An obvious approach would
be to use a PD law
:math:`\ddot x_\text{des}= k_p (x_\text{ref}- x) + k_d (\dot x_\text{ref}- \dot x)`
and translate :math:`\ddot x_\text{des}` to controls using inverse
dynamics. By choosing :math:`k_p` and :math:`k_d` appropriately one can
generate any desired damped/oscillatory-exponential approach behavior
(section `[secPD] <#secPD>`__ derives the necessary equations).

However, while PD laws are fundamental for low-level optimal control
under noise (e.g. as result of the Riccati equation), they are actually
not great for generating more macroscopic approach behavior: They are
“only” exponentially converging, never really reaching the target in a
definite time, never providing a clear expected time-to-target. And
accelerations seem intuitively too large when far from the set point,
and too small when close. (Which is why many heuristics, such as capped
PD laws were proposed.)

Instead of imposing a desired PD behavior, we can impose a desired cubic
spline behavior, which leads to succinct convergence in a finite
expected time-to-target, as well as moderate gains when far. The
approach is simply to choose an optimal :math:`\tau` (time-to-target)
that minimizes

.. math::

   \begin{aligned}
   \min_{\tau, x}~ \alpha\tau + \int_0^\tau \ddot x(t)^2~ dt\end{aligned}

under our boundary conditions, assuming a cubic spline
:math:`x(t), t\in[0,\tau]`. Using (`[eqLeap] <#eqLeap>`__), we know the
optimal :math:`x` and optimal control costs for given :math:`\tau`. When
:math:`\delta= x_\text{ref}- x` and :math:`v` are co-linear (i.e., the
system moves towards the target), computer algebra can tell us the
optimal :math:`\tau`:

.. math::

   \begin{aligned}
   \label{eqTimingControl}
     \tau^* = \frac{1}{\alpha}\Big[ \sqrt{6 |\delta| \alpha+ v^2} - |v| \Big] ~.\end{aligned}

If the system has a lateral movement, the analytical solution seems
overly complex, but a numerical solution to the least-squares form
(`[eqLeapSOS] <#eqLeapSOS>`__) very efficient. However, in practise,
using (`[eqTimingControl] <#eqTimingControl>`__) with scalar
:math:`v \gets (\delta^{\!\top\!}v)/|\delta|` for easy timing control of
convergence to a target is highly efficient and versatile.

To make this a reactive control scheme, in each control cycle
:math:`\tau^*` is reevaluated and the corresponding cubic spline
reference send to low-level control. If there are no perturbations, the
estimated :math:`\tau^*` will be the true time-to-target. See
:raw-latex:`\cite{22-toussaint-SecMPC}` for details and comparision to
PD approach behavior.

The ``moveTo`` method of ``BotOP`` uses exactly this scheme to realize
reactive control.

Hermite Cubic Splines
~~~~~~~~~~~~~~~~~~~~~

A Hermite cubic spline is specified by the series of non-decreasing time
knots, :math:`t_0,..,t_m \in [0,T]`, :math:`t_0=0, t_m=T`, and the
waypoints :math:`x_i` *and velocities* :math:`v_i` at each time knot.
There are not double knots, so the interval :math:`[0,T]` is split in
:math:`m` cubic pieces, where the :math:`i`\ th piece is determined by
the boundary conditions
:math:`(x_{i{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}},v_{i{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}, x_i, v_i)`
and
:math:`\tau_i = t_i - t_{i{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}`.

Specifying the timings (i.e., knots) and velocities of all waypoints is
often not easy in terms of a user interface. Therefore the question is
whether a series of given weypoints can easily be augmented with optimal
timings and waypoint velocities.

Further, since each piece respects boundary conditions, continuity in
velocities is ensured. However, note that two pieces might have
completely different accelerations at their joining knots (from the left
and the right), and therefore a freely specified Hermite cubic spline is
discontinuous in acceleration (has infintite jerk). Conversely,
requiring a path in :math:`{\cal C}^2` implies continuity constraints in
acceleration at each knot. Over the full path, these are
:math:`(m{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}) \cdot n`
constraints (in :math:`{\mathbb{R}}^n`), which “kill” the
degrees-of-freedom of all but the start and end velocity. Therefore,
requiring continuous accelleration, the kots, waypoints and start/end
velocity alone are sufficient to specify the spline – but in practise
the resulting waypoint velocities might not be quite desired, as they
might “go crazy” when chaining forward the continuous acceleration
constraint.

However, optimizing both, timing and waypoint velocities under out
optimal control objective is rather efficient and effective. Note that
the optimal control cost over the full spline is just the sum of single
piece costs (`[eqLeapSOS] <#eqLeapSOS>`__). This represents costs as a
least-squares of differentiable features, where :math:`D` can be
interpreted as distance to be covered by accelerations, and :math:`V` as
necessary total acceleration, and the Jacobians of :math:`\tilde D` and
:math:`\tilde V` w.r.t. all boundary conditions and :math:`\tau_i` are
trivial. Exploiting the least-squares formulation of :math:`\psi` we can
use the Gauss-Newton approximate Hessian.

As a concequence, it is fairly efficient to solve for
:math:`\tau_{1:m}`,
:math:`v_{1:m{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}` given
:math:`v_0, v_m, x_{0:m}` under continuous acceleration constraints
subject to total time and control costs.

The C++ code implementes this with the ``TimingOpt`` class, leverging
our standard AugLag method and the least-squares formulation
(`[eqLeapSOS] <#eqLeapSOS>`__).

As a final note, in Hermite quintic splines we need positions
:math:`x_i`, velocities :math:`v_i` and accelerations :math:`a_i` at
each knot, which describe the quintic polynomial pieces between knots.
The issues discussed above apply here analogously.

B-Splines
~~~~~~~~~

In B-splines, the path :math:`x: [0,T] \to {\mathbb{R}}^n` is expressed
as a linear combination of control points
:math:`z_0,.., z_K \in {\mathbb{R}}^n`,

.. math::

   \begin{aligned}
   \label{bspline}
   x(t) = \sum_{i=0}^K B_{i,p}(t)~ z_i ~,\end{aligned}

where :math:`B_{i,p}: {\mathbb{R}}\to {\mathbb{R}}` maps the time
:math:`t` to the weighting of the :math:`i`\ th control point – it
blends in and out the :math:`i`\ th control point. For any :math:`t` it
holds that :math:`\sum_{i=0}^K B_{i,p}(t) = 1`, i.e., all the weights
:math:`B_{i,p}(t)` sum to one (as with a probability distribution over
:math:`i`), and the path point :math:`x(t)` is therefore always in the
convex hull of control points.

Concerning terminology, actually the functions :math:`B_{i,p}(t)` are
called **B-splines**, not the resulting path :math:`x(t)`. (But in
everyday robotics language, one often calls the path a B-spline.) As the
linear (scalar) product in (`[bspline] <#bspline>`__) is trivial, the
maths (and complexity of code) is all about the B-splines
:math:`B_{i,p}(t)`, not the path :math:`x(t)`.

The B-spline functions :math:`B_{i,p}(t)` are fully specified by a
non-decreasing series of time knots :math:`t_0,..,t_m \in [0,T]` and the
integer degree :math:`p\in\{0,1,..\}`. Namely, the recursive definition
is

.. math::

   \begin{aligned}
   B_{i,0}(t) &= [t_i \le t < t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}] ~,\quad\text{for $0\le i \le m-1$} ~,\\
   B_{i,p}(t)
   &= \frac{t-t_i}{t_{i+p}-t_i}~ B_{i,p-1}(t)
    +  \frac{t_{i+p+1}-t}{t_{i+p+1}-t_{i+1}}~ B_{i+1,p-1}(t)  ~,\quad\text{for $0\le i \le m-p-1$} ~.\end{aligned}

The zero-degree B-spline functions :math:`B_{i,0}` are indicators of
:math:`t_i \le t < t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}`,
and :math:`i` ranges from :math:`i=0,..,m-1`. The 1st-degree B-spline
functions :math:`B_{i,1}` have support in :math:`t_i \le t < t_{i+2}`
and :math:`i` only ranges in :math:`i=0,..,m-2` – because one can show
that the normalization :math:`\sum_{i=0}^{m-2} B_{i,1}(t) = 1` holds
(and for :math:`i>m-2`, the recursion would also not be clearly
defined). In general, degree :math:`p` B-spline functions
:math:`B_{i,p}` have support in :math:`t_i \le t < t_{i+p+1}` and
:math:`i` ranges from :math:`i=0,..,m+p-1`, which is why we need
:math:`K+1` control points :math:`z_{0:K}` with

.. math::

   \begin{aligned}
       K = m+p-1 ~,
     \end{aligned}

which ensures the normalization property
:math:`\sum_{i=0}^K B_{i,p}(t) = 1` for every degree.

|image|

[figSplines] Illustration of B-spline functions for degrees
:math:`p=0,..,4`. Above each plot of functions, a rough illustration of
a resulting spline is provided, where bullets indicate control points.
Note that this illustration implies a localization of control points in
time, namely roughly where the coresponding weighting function (B-spline
function) is highest – but control points are formally not localized in
time, they are just being linearly combined,
:math:`x(t) = \sum_{i=0}^K B_{i,p}(t)~ z_i`, with different weighting in
time. However, intuitively we can see that for odd degrees, the
“localization in time” of control points roughly aligns with knots,
while for even degrees the localization is between knots. Further, the
illustrations assume multi-knots at the start and end (namely
:math:`p{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}`-fold knots),
which ensures that the spline starts with :math:`z_0` and ends with
:math:`z_K`. Multiple equal control points :math:`z_{0:p}` and
:math:`z_{K-p:K}` (illustrated with gray bars) are needed to ensure also
zero vel/acc/jerk at start and end.

B-spline Matrix for Time Discretized Paths
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Splines describe a continuous path :math:`x(t)`, but often we want to
evaluate this path only at a finite number of time slices
:math:`t\in \{\widehat t_1,..,\widehat t_S\} \subset [0,T]`. E.g., this
could be a grid of :math:`S=100` time slices over which we want to
optimize using KOMO, and for which we have to compute collision
features. Let :math:`x \in {\mathbb{R}}^{S \times n}` be the time
discretized path, and
:math:`z \in{\mathbb{R}}^{K{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}\times n}`
be the stack of control points. Then the B-spline representation becomes

.. math::

   \begin{aligned}
   x = B_p z ~,\quad\text{with } B_p\in{\mathbb{R}}^{S\times K{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}},~ B_{p,si} = B_{i,p}(\widehat t_s) ~,\end{aligned}

where :math:`B_p` is the B-spline matrix of degree :math:`p` for this
particular time grid :math:`\{\widehat t_1,..,\widehat t_S\}`.

So whenever we have a problem (e.g., NLP) defined over the fine
resolution samples :math:`x_s`, the B-spline matrix provides a linear
re-parameterization and it is trivial to pull gradients (and Hessians)
back to define a problem over :math:`z`. In our code, KOMO defines NLPs
over trajectories – it is trivial to wrap this with a linear B-spline
parameterization to then imply a much lower-dimensional NLP over the
control points :math:`z`.

Ensuring B-splines pass through waypoints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As we emphasized, the control point parameterization is not necessarily
intuitive for a user, as the resulting path does not transition through
control points. If a user provides a series of waypoints at desired
times :math:`\widehat t_s`, how can we construct a B-spline to ensure
transitioning through these waypoints at the desired times?

The answer is again the matrix equation. Consider the cubic spline case
and that the start and end points and times are fixed. Therefore
:math:`z_{0:1}` and
:math:`z_{K{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}:K}`, as well as
knots :math:`t_{0:3}` and :math:`t_{m-3:m}` are fixed. The user wants
waypoints :math:`x_1,..,x_S` at times
:math:`\widehat t_1,..,\widehat t_S` *between* start and end.

We can distribute :math:`S` knots :math:`t_{4:3+S}` uniformly between
start and end knots (or also at :math:`\widehat t_1,..,\widehat t_S`),
from which it follows we have :math:`m = S+7`, and :math:`K=m-p-1=S+3`,
which are :math:`K+1=S+4` control points in total, of which :math:`4`
are already fixed. So the :math:`S` middle control points are still
free, and matrix inversion gives them from the desired waypoints,

.. math::

   \begin{aligned}
     z_{2:S+1} = B^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1} x_{1:S} ~,\quad\text{with } B \in {\mathbb{R}}^{S \times S},~ B_{si} =  B_{i+1,3}(\widehat t_s),~ s,i=1,..,S  ~.
     \end{aligned}

Ensuring boundary velocities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Consider again an online control situation where the is in state
:math:`(x,\dot x)` and we want to steer it through future waypoints. In
the B-spline representation we have to construct a spline that starts
with current state as starting boundary.

For degrees 2 and 3 this is simple to achieve: In both cases we usually
have :math:`z_0=z_1` and
:math:`z_{K{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}=z_K` to ensure
zero start and end velocities. Modifying :math:`z_1` directly leads to
the start velocity
:math:`\dot x(0) = \dot B_{0,p}(0) z_0 + \dot B_{1,p}(0) z_1`. But
because of normalization we have
:math:`\dot B_{0,p}(0) = - \dot B_{1,p}(0)`, and therefore

.. math::

   \begin{aligned}
     \dot x(0) &= \dot B_{0,p}(0) (z_0 - z_1) \\
     z_1 &= z_0 - \frac{\dot x(0)}{\dot B_{0,p}(0)} ~.\end{aligned}

Gradients
^^^^^^^^^

The gradients of a B-spline represented path w.r.t. control points are
trivial. But the gradients w.r.t. the knots are less trivial. Here the
basic equations:

.. math::

   \begin{aligned}
   B_{i,p}(t)
     &= \frac{t-t_i}{t_{i+p}-t_i} B_{i,p-1}(t)
    +  \frac{t_{i+p+1}-t}{t_{i+p+1}-t_{i+1}} B_{i+1,p-1}(t) \\
   &=: v~ B_{i,p-1} + w~ B_{i+1,p-1} \\
   \dot B_{i,p}(t)
    &= \frac{1}{t_{i{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}{}p}-t_i}~ B_{i,p{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}(t)
    + v~ \dot B_{i,p{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}(t)
    - \frac{1}{t_{i{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}{}p{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}-t_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1}}}~ B_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1},p{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}(t)
    + w~ \dot B_{i{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}1},p{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}}(t) \\
   \partial_{t_i} B_{i,p}
     &= \Big[\frac{-1}{t_{i+p}-t_i} + \frac{t-t_i}{(t_{i+p}-t_i)^2}\Big]~ B_{i,p-1}
      + v~ \partial_{t_i} B_{i,p-1} + w~ \partial_{t_i} B_{i+1,p-1} \\
     &= \Big[\frac{-1}{t-t_i} + \frac{1}{t_{i+p}-t_i} \Big]~ v~ B_{i,p-1} + v~ \partial_{t_i} B_{i,p-1} + w~ \partial_{t_i} B_{i+1,p-1} \\
   \partial_{t_{i+1}} B_{i,p}
     &= \Big[\frac{1}{t_{i+p+1}-t_{i+1}}\Big]~ w~ B_{i+1,p-1}
         + v~ \partial_{t_{i+1}}~ B_{i,p-1} + w~ \partial_{t_{i+1}}~ B_{i+1,p-1} \\
   \partial_{t_{i+p}} B_{i,p}
     &= \Big[- \frac{1}{t_{i+p}-t_{i}}\Big]~ v~ B_{i,p-1}
         + v~ \partial_{t_{i+p}}~ B_{i,p-1} + w~ \partial_{t_{i+p}}~ B_{i+1,p-1} \\
   \partial_{t_{i+p+1}} B_{i,p}
     &= \Big[\frac{1}{t_{i+p+1}-t} - \frac{1}{t_{i+p+1}-t_{i+1}}\Big]~ w~ B_{i+1,p-1}
         + v~ \partial_{t_{i+p+1}}~ B_{i,p-1} + w~ \partial_{t_{i+p+1}}~ B_{i+1,p-1}\end{aligned}

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

The Editing Configurations tutorial
<https://marctoussaint.github.io/robotic/tutorials/config_3_import_edit.html>
shows this file syntax is used to specify robot/environment
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

-  image coordinates :math:`u=(u_x,u_y,u_z)`, with the pixel coordinates
   :math:`(u_x,u_y)` and depth coordinate :math:`u_z`, details as
   followed.

The pixel coordinates :math:`(u_x,u_y)` indicate where a point appears
on the image plane. The :math:`x`-axis always points to the right, but
there are two conventions for the :math:`y`-axis:

-  :math:`y`-up: The :math:`y`-axis points upward. This is consistent to
   how a diagram is typically drawn on paper: :math:`x`-axis right,
   :math:`y`-axis up. However, a consequence is that the :math:`z`-axis
   then points backward, i.e., pixels in front of the camera have
   negative depth :math:`u_z`.

-  :math:`y`-down: The :math:`y`-axis points down. This is consistent to
   how pixels are typically indexed in image data: counting rows from
   top to bottom. So when defining pixel coordinates :math:`(u_x,u_y)`
   literally to be pixel indices in image data, :math:`y`-down is the
   natural convention. A consequence is that the :math:`z`-axis points
   forward, i.e., pixels in front of the camera have a positive depth
   :math:`u_z`, which might also be more intuitive.

The transformation from camera coordinates :math:`x` to image
coordinates :math:`u` is involves perspective projection. For better
readability, let’s write
:math:`x \equiv (\texttt{x},\texttt{y},\texttt{z})`. Then the mapping is

.. math::

   \begin{aligned}
   \label{eqxtou}
   u =  \left(\begin{array}{c}u_x \\ u_y \\ u_z\end{array}\right) 
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

Given camera coordinates
:math:`x \equiv (\texttt{x},\texttt{y},\texttt{z})`, we can write
(`[eqxtou] <#eqxtou>`__)

.. math::

   \begin{aligned}
   \boldsymbol u
   &= K x
   =  \left(\begin{array}{c}f_x \texttt{x}+ s \texttt{y}+ c_x \texttt{z}\\ f_y \texttt{y}+ c_y \texttt{z}\\ \texttt{z}\end{array}\right)  ~,\quad
   K =  \left(\begin{array}{ccc}f_x & s & c_x \\ & f_y & c_y \\ & & 1 \end{array}\right)  ~,\quad
   {\cal P}(\boldsymbol u)
   =  \left(\begin{array}{c}  (f_x x + s \texttt{y})/\texttt{z}+ c_x\\ f_y \texttt{y}/\texttt{z}+ c_y \end{array}\right)  ~,\end{aligned}

where :math:`\boldsymbol u` are *homogeneous pixel* coordinates, and
:math:`{\cal P}(\boldsymbol u)` the actual pixel coordinates, which
would have to be augmented with :math:`\texttt{z}` again to get the
:math:`u` including depth coordinate.

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
entries :math:`\boldsymbol u = (u_x u_z, u_y u_z, u_z)` which includes
the true depth :math:`u_z`. So this method is only applicable when we
want to calibrate a depth camera.

Given data :math:`D = \{(\boldsymbol u_i, \boldsymbol X_i)\}_{i=1}^n`,
we want to minimize the squared error

.. math::

   \begin{aligned}
   \text{argmin}_P \sum_i (\boldsymbol u_i - P \boldsymbol X_i)^2 = [U - P X]^2 ~,\end{aligned}

where :math:`U` and :math:`X` are the stacked :math:`\boldsymbol u_i`
and :math:`\boldsymbol X_i`, respectively. The solution is
:math:`P = U^{\!\top\!}X (X^{\!\top\!}X)^{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}1}`.
Comparing with the definition
:math:`P=  \left(\begin{array}{cc}KR^{\!\top\!}& -KR^{\!\top\!}t\end{array}\right)`,
we can decompose it and extract explicit :math:`K, R, t` using

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

