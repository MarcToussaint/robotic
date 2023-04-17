#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import sys, os
#sys.path.append(os.path.expanduser('~/git/rai-python/build'))
#import libry as ry
from robotic import ry
#ry.setRaiPath(os.path.expanduser('~/git/rai-python/rai-robotModels'))


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view()


# In[ ]:


C.addFrame('box','table')     .setRelativePosition([-.25,.1,.1])     .setShape(ry.ST.ssBox, size=[.1,.1,.1,.02])     .setColor([1,.5,0])
C.view()


# In[ ]:


komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(1., 1, 5., 0)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([], ry.FS.positionDiff, ['l_gripper', 'box'], ry.OT.eq, [1e1]);


# In[ ]:


ret = ry.NLP_Solver()     .setProblem(komo.nlp())     .setOptions( stopTolerance=1e-2, verbose=4 )     .solve()
print(ret)


# In[ ]:


komo.view(False, "IK solution")


# In[ ]:


q = komo.getPath()
print(type(q), len(q))


# In[ ]:


C.setJointState(q[0])
C.view()


# In[ ]:




