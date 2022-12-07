import sys
sys.path.append('../build')
import time
import numpy as np
#from robotic import ry
import libry as ry

## create a configuration
C = ry.Config()
C.addFile("../rai-robotModels/pr2/pr2.g")
C.addFile("../rai-robotModels/objects/tables.g")
C.addFrame("obj0", "table1", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.], contact, logical={ object }, joint:rigid, Q:<t(0 0 .15)>" )
C.addFrame("obj1", "table1", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.], contact, logical={ object }, joint:rigid, Q:<t(0 .2 .15)>" )
C.addFrame("obj2", "table1", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.], contact, logical={ object }, joint:rigid, Q:<t(0 .4 .15)>" )
C.addFrame("obj3", "table1", "type:ssBox size:[.1 .1 .2 .02] color:[1. 0. 0.], contact, logical={ object }, joint:rigid, Q:<t(0 .6.15)>" )
C.addFrame("tray", "table2", "type:ssBox size:[.15 .15 .04 .02] color:[0. 1. 0.], logical={ table }, Q:<t(0 0 .07)>" );
C.addFrame("", "tray", "type:ssBox size:[.27 .27 .04 .02] color:[0. 1. 0.]" )
#C.view(False, "initial model")

## create a skeleton
S = ry.Skeleton()
S.addEntry([1.], ry.SY.touch, ['pr2R', 'obj0'])
S.addEntry([1., 3.], ry.SY.stable, ['pr2R', 'obj0'])
S.addEntry([2.,2.], ry.SY.touch, ['pr2L', 'obj3'])
S.addEntry([2.,4.], ry.SY.stable, ['pr2L', 'obj3'])
S.addEntry([3.], ry.SY.above, ['obj0', 'tray'])
S.addEntry([3.,4.], ry.SY.stableOn, ['tray', 'obj0'])

## solve for waypoints: create a komo instance, create nlp instance, then call generic solver
komo = S.getKomo_waypoints(C, 1e-1, 1e-2)
nlp = komo.nlp()
sol = ry.NLP_Solver()
sol.setProblem(nlp)
sol.setOptions( stopTolerance=1e-2 )
ret = sol.solve()
waypoints = komo.getPath_qAll()
# report on result, view, and play
print(ret)
#print(nlpW.report(2))
komo.view(True, "waypoints solution")
komo.view_play(True, .2)
# store result

## solve for paths using RRT: for each phase create start-end problems, run RRT
m = len(waypoints)
rrt_dofs = []
rrt_paths = []
for t in range(0,int(m)):
    # grab config and waypoints
    [Ctmp, q0, q1] = S.getTwoWaypointProblem(t, komo)
    Ctmp.setJointState(q0);
    Ctmp.view(True, "waypoint configuration phase " + str(t) + " START")
    Ctmp.setJointState(q1);
    Ctmp.view(True, "waypoint configuration phase " + str(t) + " STOP")

    # call path finder
    sol = ry.PathFinder()
    sol.setProblem(Ctmp, q0, q1)
    ret = sol.solve()
    rrt_paths.append(ret.x)
    rrt_dofs.append(Ctmp.getDofIDs())

    #display the rrt path
    for i in range(0,ret.x.shape[0]):
        Ctmp.setJointState(ret.x[i])
        Ctmp.view(False, "rrt path " + str(i))
        time.sleep(.02)

## solve for full path: create a komo instance, initialize with waypoints & rrt paths, solve
komo = S.getKomo_path(C, 20, .2, -1, 1e-2)
komo.initWithWaypoints(waypoints)
komo.view(True, "init with waypoints only")
for t in range(0,int(m)):
    komo.initPhaseWithDofsPath(t, rrt_dofs[t], rrt_paths[t], True)
    komo.view(True, "init with RRT phase " + str(t))
nlp = komo.nlp()
sol = ry.NLP_Solver()
sol.setProblem(nlp)
sol.setOptions( stopTolerance=1e-2 )
ret = sol.solve()
# report on result, view, and play
print(ret)
#print(nlp.report(2))
komo.view(True, "path solution")
komo.view_play(True, .2)
