import os
import numpy as np
import robotic as ry
from lxml import etree


class URDFLoader():

    def __init__(self, file, visualsOnly=True, meshPathRemove='', meshExt=None):
        self.meshPathRemove = meshPathRemove
        self.meshExt = meshExt

        xmlData = etree.parse(file)
        self.path, _ = os.path.split(file)

        self.C = ry.Config()

        useCollisionShapes = not visualsOnly

        # materials

        self.materials = {}    
        mats = xmlData.findall('./material')
        for mat in mats:
            name = mat.attrib.get('name', '')
            tex = mat.find('texture')
            if tex is not None:
                tex_file = tex.attrib['filename'].replace(self.meshPathRemove,'',1)
                self.materials[name] = os.path.join(self.path, tex_file)
            else:
                col = mat.find('color')
                if col is not None:
                    self.materials[name] = self.as_floats(col.attrib['rgba'])

        # links with shapes

        links = xmlData.findall('./link')
        for link in links:
            link_name = link.attrib['name']
            f_link = self.C.addFrame(link_name)


            elem = link.find('inertial/inertia')
            matrix=[]
            if elem is not None:
                tags = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
                matrix = [float(elem.attrib[tag]) for tag in tags]

            elem = link.find('inertial/mass')
            if elem is not None:
                mass = float(elem.attrib['value'])
                f_link.setMass(mass, matrix)

            for visual in link.findall('visual'):
                self.add_shape(visual, f'{link_name}_0', f_link, isVisual=True)

            # collision shape
            if useCollisionShapes:
                for collision in link.findall('collision'):
                    f_shape = self.add_shape(collision, f'{link_name}_1', f_link, isVisual=False)
                    f_shape.setColor([1,0,0,.2])

        # joints

        joints = xmlData.findall('./joint')
        for joint in joints:
            joint_name = joint.attrib['name']
            if joint.find('child') is None:
                continue

            parent_name = joint.find('parent').attrib['link']
            f_parent = self.C.getFrame(parent_name)
            if f_parent is None:
                print('SHIT', joint_name)
                break

            # add an origin frame as pre frame?
            elem = joint.find('origin')
            if elem is not None:
                f_origin = self.C.addFrame(f'{joint_name}_origin')
                f_origin.setParent(f_parent)
                self.setRelativePose(f_origin, elem)
                f_parent = f_origin

            f_joint = self.C.addFrame(joint_name)
            f_joint.setParent(f_parent)

            child_name = joint.find('child').attrib['link']
            f_child = self.C.getFrame(child_name)
            if f_child is None:
                print('SHIT', joint_name)
                break
            f_child.setParent(f_joint, False)

            elem = joint.find('limit')
            limits = []
            if elem is not None:
                lo = elem.attrib.get('lower')
                up = elem.attrib.get('upper')
                eff = elem.attrib.get('effort')
                vel = elem.attrib.get('velocity')
                if eff=='0':
                    eff=None
                if vel=='0':
                    vel=None
                if lo is not None:
                    limits = [float(lo), float(up)]
                # if vel is not None:
                    # print(' ctrl_limits: [%s -1 %s],' % (vel, eff), end='') #the 2nd value is an acceleration limit
            # else:
            #     elem = joint.find('safety_controller')
            #     if elem is not None:
            #         lo = elem.attrib.get('soft_lower_limit')
            #         up = elem.attrib.get('soft_upper_limit')
            #         if lo is not None:
            #             print(' limits: [%s %s],' % (lo, up), end='')

            elem = joint.find('mimic')
            f_mimic = None
            if elem is not None:
                f_mimic = self.C.getFrame(elem.attrib['joint'])
                if f_mimic is None:
                    print('SHIT', elem.attrib['joint'])
                    break

            att = joint.attrib.get('type')
            
            if att in ['revolute', 'continuous']:
                elem = joint.find('axis')
                if elem is not None:
                    axis = elem.attrib['xyz']
                    if axis=='1 0 0':
                        f_joint.setJoint(ry.JT.hingeX, limits, 1., f_mimic)
                    elif axis=='0 1 0':
                        f_joint.setJoint(ry.JT.hingeY, limits, 1., f_mimic)
                    elif axis=='0 0 1':
                        f_joint.setJoint(ry.JT.hingeZ, limits, 1., f_mimic)
                    elif axis=='0 0 -1':
                        f_joint.setJoint(ry.JT.hingeZ, limits, -1., f_mimic)
                    else:
                        raise Exception('CAN ONLY PROCESS X Y Z hinge joints, not', axis)
                else:
                    f_joint.setJoint(ry.JT.hingeX, limits, 1., f_mimic)
                    
            if att == 'prismatic':
                elem = joint.find('axis')
                if elem is not None:
                    axis = elem.attrib['xyz']
                    if axis=='1 0 0':
                        f_joint.setJoint(ry.JT.transX, limits, 1., f_mimic)
                    elif axis=='0 1 0':
                        f_joint.setJoint(ry.JT.transY, limits, 1., f_mimic)
                    elif axis=='0 -1 0':
                        f_joint.setJoint(ry.JT.transY, limits, -1., f_mimic)
                    elif axis=='0 0 1':
                        f_joint.setJoint(ry.JT.transZ, limits, 1., f_mimic)
                    elif axis=='0 0 -1':
                        f_joint.setJoint(ry.JT.transZ, limits, -1., f_mimic)
                    else:
                        raise Exception('CAN ONLY PROCESS X Y Z prismatic joints, not', axis)
                else:
                    f_joint.setJoint(ry.JT.transX, limits, 1., f_mimic)

            if att == 'fixed':
                f_joint.setJoint(ry.JT.rigid)

            #elem = joint.find('axis')
            #if elem is not None:
            #    print('axis:[%s]' % elem.attrib['xyz'])

    def add_shape(self, shape, shape_name, f_link, isVisual):
        f_shape = self.C.addFrame(shape_name)
        f_shape.setParent(f_link)

        elem = shape.find('origin')
        if elem is not None:
            self.setRelativePose(f_shape, elem)

        elem = shape.find('geometry/box')
        if elem is not None:
            size = self.as_floats(elem.attrib['size'])
            f_shape.setShape(ry.ST.box, size)

        elem = shape.find('geometry/sphere')
        if elem is not None:
            size = self.as_floats(elem.attrib['size'])
            f_shape.setShape(ry.ST.sphere, size)

        elem = shape.find('geometry/cylinder')
        if elem is not None:
            size = [float(elem.attrib[tag]) for tag in ['length', 'radius']]
            f_shape.setShape(ry.ST.cylinder, size)

        elem = shape.find('geometry/mesh')
        if elem is not None:
            filename = elem.attrib['filename'].replace(self.meshPathRemove,'',1)
            if self.meshExt is not None:
                filename = filename[:-3] + self.meshExt
            if 'scale' in elem.attrib:
                scale = self.as_floats(elem.attrib['scale'])[0]
            else:
                scale = 1
            filename = os.path.join(self.path, filename)
            f_shape.setMeshFile(filename, scale)

        elem = shape.find('material/color')
        if elem is not None:
            f_shape.setColor(self.as_floats(elem.attrib['rgba']))

        # elem = shape.find('material')
        # if elem is not None and 'name' in elem.attrib:
        #     mat = self.materials[elem.attrib['name']]
        #     if len(mat)==4: #rgba
        #         f_shape.setColor(mat)
        #     else:
        #         f_shape.setTextureFile(mat)

        
        return f_shape

    def as_floats(self, input_string):
        return [float(num) for num in input_string.replace(',', ' ').split()]
    
    def setRelativePose(self, f, attrib):
        pos = attrib.get('xyz')
        if pos=='0 0 0':
            pos=None
        if pos is not None:
            f.setRelativePosition(self.as_floats(pos))

        rpy = attrib.get('rpy')
        if rpy=='0 0 0':
            rpy=None
        if rpy is not None:
            q = ry.Quaternion()
            q.setRollPitchYaw(self.as_floats(rpy))
            f.setRelativeQuaternion(q.getArr())
