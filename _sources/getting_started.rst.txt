Getting Started
===============

Quick Install
-------------

Install the ``robotic`` python package. On a standard Ubuntu, the
following should be sufficient:

::

   sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
   python3 -m pip install --user robotic

A standard test is

::

   python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'
   python3 -c 'import robotic as ry; ry.test.RndScene()'

Compiling from Source & Docstrings
----------------------------------

-  **Compiling for the real robot:** The ``robotic`` pip package does
   not have the real robot drivers (for the Franka Panda robot)
   included, and therefore only supports simulation mode. However, the
   interface and code is designed so that everything directly transfers
   to the real robot: All you have to do is re-compile the python
   package on your machine, including the libfranka and librealsense
   drivers. This is done by installing the
   `robotic <https://github.com/MarcToussaint/robotic>`__ package.
-  **Docstrings and tab completion:** The package comes with a ‘stubs’
   file (should be in
   ``~/.local/lib/python*/site-packages/robotic/_robotic.ipy``), which
   includes docstrings most methods and should enable autocompletion in
   your IDE. The docstrings are not yet great for all methods.

   If tab-autocomplete for jupyter does not work, try
   ``python3 -m pip install --user jedi==0.17.2``

Downloading the tutorial notebooks
----------------------------------

::

   git clone https://github.com/MarcToussaint/rai-tutorials.git
   cd rai-tutorials
   jupyter-notebook .

If you don’t have jupyter installed yet:

::

   pip3 install --user jupyter nbconvert matplotlib
