#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import sys, os
#sys.path.append(os.path.expanduser('~/git/rai-python/build'))
#import libry as ry
from robotic import ry
import time


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
q0 = C.getJointState()
C.view()


# In[ ]:


C.addFrame('boxR','table')     .setRelativePosition([.15,0,.1])     .setShape(ry.ST.ssBox, size=[.1,.1,.1,.02])     .setColor([1,1,0])
C.addFrame('boxL','table')     .setRelativePosition([-.15,0,.1])     .setShape(ry.ST.ssBox, size=[.1,.1,.1,.02])     .setColor([1,.5,0])
C.view()


# In[ ]:


komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(1., 20, 5., 2)
komo.addControlObjective([], 2, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([1.], ry.FS.positionDiff, ['r_gripper', 'boxL'], ry.OT.eq, [1e1]);
komo.addObjective([1.], ry.FS.positionDiff, ['l_gripper', 'boxR'], ry.OT.eq, [1e1]);
komo.addObjective(times=[1.], feature=ry.FS.qItself, type=ry.OT.eq, scale=[1e1], order=1);


# In[ ]:


ret = ry.NLP_Solver()     .setProblem(komo.nlp())     .setOptions( stopTolerance=1e-2, verbose=4 )     .solve()
print(ret)


# In[ ]:


komo.view(True, "path opt solution")
komo.view_play()


# In[ ]:


print('ret.x returns:', type(ret.x), ret.x.shape)
path = komo.getPath()
print('getPath returns:', type(path), len(path), path[0].shape)


# In[ ]:


# display the path
for t in range(0, len(path)):
    C.setJointState(path[t])
    C.view()
    time.sleep(.1)


# In[ ]:


# run the path with botop
C.setJointState(q0)
ry.params_add({'botsim/verbose': 1., 'physx/motorKp': 10000., 'physx/motorKd': 1000.})
bot = ry.BotOp(C, False)
bot.home(C)


# In[ ]:


bot.moveAutoTimed(path, 1., 1.)
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


# In[ ]:


del bot


# In[ ]:




