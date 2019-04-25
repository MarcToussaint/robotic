## Tutorials

Only a few of the tutorials exist yet. Please see the also [docs/](docs/) path. The plan is:

1. [Basics:](1-basics.ipynb) Configurations, Features & Jacobians
1. [Features:](2-features.ipynb) Learn about the language to define and query features and their Jacobians. Including querying collision features (whether and which objects are in collision).
1. [IK:](3-IK-optimization.ipynb) The simplest use of KOMO is for inverse kinematics - learn how to add features to an optimization problem
1. [KOMO:](4-path-optimization.ipynb) Proper path optimization examples
1. [CGO:](5-cgo-optimization.ipynb) KOMO can also used in "dense" mode, where it optimize as constraint graph
1. [Skeletons:](6-KOMO-skeleton.ipynb) Instead of specifying features low-level, you can specify a skeleton and query a pre-defined bound for that skeleton (path, or sequence). This is kind of a higher level language to set objectives. (But not as general as low-level features. You can mix both.)
1. [Robot Models:](9-robotModels.ipynb) Some info on which scene/robot models are available and how to convert from URDF
1. [LGP:](lgp1-pickAndPlace.ipynb) The first full LGP demo - for now only for pickAndPlace
1. [bullet:](sim1-bullet.ipynb) calling bullet (requires to comment 'BULLET=0' in [config.mk](../config.mk) before compilation)
