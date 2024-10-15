import robotic as ry
import time

C = ry.Config()
C.addFile(ry.raiPath('panda/panda.g'), 'a_').setPosition([1,0,.5])
C.addFile(ry.raiPath('panda/panda.g'), 'b_').setPosition([0,0,.5])

C.addFrame('box1') .setShape(ry.ST.ssBox,[.3, .3, .3, .05]) .setPosition([1.3, 0, 2.]) .setMass(3)
C.addFrame('box2') .setShape(ry.ST.ssBox,[.3, .3, .3, .05]) .setPosition([.5, .0, 1.8]) .setMass(3)

C.view(False)

q0 = C.getJointState()
qT = q0
qT[0] += 1.

sim = ry.Simulation(C, ry.SimulationEngine.physx, verbose=1) #try verbose=2

[X, q, V, qDot] = sim.getState()

tau=.01

sim.setSplineRef(qT, [1.])

for t in range(int(4./tau)):
    time.sleep(tau)

    sim.step([], tau, ry.ControlMode.spline)

    if (t%100)==0:
        sim.setState(X, q, V, qDot)
        q0 = q
        sim.resetSplineRef()
        sim.setSplineRef(qT, [1.])
