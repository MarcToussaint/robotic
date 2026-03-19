import robotic as ry
import math

def create_grid_config(C, n, dist):
    Cgrid = ry.Config()
    nx = ny = int(math.ceil(math.sqrt(n)))
    x,y = 0,0
    for i in range(n):
        base = Cgrid.addConfigurationCopy(C, f'{i}_') #the string adds a prefix to all frame names
        # move all root frames
        for j in range(base.ID, Cgrid.getFrameDimension()):
            f = Cgrid.frame(j)
            if f.getParent() is None:
                p = f.getPosition()
                p[0] += dist*(x-(nx-1)/2)
                p[1] += dist*(y-(ny-1)/2)
                f.setPosition(p)
        y += 1
        if y>=ny:
            x += 1
            y=0

    # fix the name and position of the first camera_init frame
    for f in Cgrid.getFrames():
        if 'camera_init' in f.name:
            f.name = 'camera_init'
            p = f.getPosition()
            p[0] += dist*(nx-1)/2
            p[1] += dist*(ny-1)/2
            f.setPosition(p)
            break

    # old: make all free frame relative to zero origin for mj; but don't do that anymore!
    # q_org = Cgrid.getJointState()
    # for f in Cgrid.getFrames():
    #     if 'obj' in f.name and f.getJointType()==ry.JT.free:
    #         f.setParent(f_origin, True)
    # qpos_offset = Cgrid.getJointState() - q_org
    # open('z.yml', 'w').write(Cgrid.asYaml())

    return Cgrid #, qpos_offset
