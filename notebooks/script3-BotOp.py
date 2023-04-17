#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import sys, os
#sys.path.append(os.path.expanduser('~/git/rai-python/build'))
#import libry as ry
from robotic import ry
import numpy as np
import time


# In[ ]:


ry.params_add({'botsim/verbose': 1., 'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False, 'this is your workspace data structure C -- NOT THE SIMULTATION')


# In[ ]:


bot = ry.BotOp(C, False)
#note that in sim, arms are going down! free floating...


# In[ ]:


# we need to control it somehow, e.g. to home
bot.home(C)


# In[ ]:


qHome = bot.get_q()
q = bot.get_q()
print(q)
q[1] = q[1] - .1
print(q)


# In[ ]:


bot.moveTo(q, 2)
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)


# In[ ]:


bot.home(C)


# In[ ]:


bot.gripperOpen(ry._left)


# In[ ]:


bot.gripperClose(ry._left)


# In[ ]:


bot.sync(C, .0)


# In[ ]:


del bot

