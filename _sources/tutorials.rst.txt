=========
Tutorials
=========

All tutorials are jupyter notebooks, downloadable from https://github.com/MarcToussaint/rai-tutorials. For installation instructions, see `Getting Started <getting_started.html>`_.

The first 3 are **intro tutorials** that introduce essentials for understanding the code:

* **Configurations**: The core data structure used to represent scenes and robots, compute features, and feed into optimization problems, simulations, and real robot control.
* **BotOp**: The *Robot Operation* interface used to control a real or simulated robot, as well as to access sensor.
* **KOMO**: A framework to formulate optimization problems, esp. for motion design (IK, path optimization, and manipulation planning).

.. toctree::
   :glob:
   :maxdepth: 1

   tutorials/intro_Configurations
   tutorials/intro_BotOp
   tutorials/intro_KOMO

The remaining tutorials cover various topics in more depth:

.. toctree::
   :glob:
   :maxdepth: 1

   tutorials/real_robot
   tutorials/features
   tutorials/configuration_importing_editing
   tutorials/komo_reporting
   tutorials/simulation
   tutorials/rendering
   tutorials/RRT_example
   tutorials/nlp_solving
   
