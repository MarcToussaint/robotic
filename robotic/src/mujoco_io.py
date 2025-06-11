
import os
import numpy as np
import robotic as ry
import xml.etree.ElementTree as ET
from copy import copy

class MujocoLoader():

    def __init__(self, file, visualsOnly=True, defaultConType='0', basePos=[0,0,0], baseQuat=[0,0,0,1]):
        ry.params_add({'cd_into_mesh_files': False})

        self.visualsOnly = visualsOnly
        self.defaultConType = defaultConType
        self.debug_counter = 0
        self.muj2rai_joint_map = {
            '1 0 0': ry.JT.hingeX,
            '0 1 0': ry.JT.hingeY,
            '0 0 1': ry.JT.hingeZ,
            '-1 0 0': ry.JT.hingeX,
            '0 -1 0': ry.JT.hingeY,
            '0 0 -1': ry.JT.hingeZ,
        }

        tree = ET.parse(file)
        path, _ = os.path.split(file)
        root = tree.getroot()

        self.materials = {}    
        self.textures = {}
        self.meshes = {}
        self.load_assets(root, path)
        self.bodyCount = -1

        self.C = ry.Config()
        self.base = self.C.addFrame('base')
        self.base.setAttributes({'multibody':True})
        self.base.setPosition(basePos)
        self.base.setQuaternion(baseQuat)
        self.add_node(root, self.base, path, 0)

    def as_floats(self, input_string):
        return [float(num) for num in input_string.replace(',', ' ').split()]
    
    def load_assets(self, root, path):
        texs = root.findall('.//texture')
        for tex in texs:
            name = tex.attrib.get('name', '')
            file = tex.attrib.get('file', '')
            self.textures[name] = os.path.join(path, file)

        maters = root.findall('.//material')
        for mater in maters:
            name = mater.attrib.get('name', '')
            color = mater.attrib.get('rgba', '')
            texture_name = mater.attrib.get('texture', '')
            if texture_name:
                self.materials[name] = self.textures[texture_name]
            elif color:
                self.materials[name] = color
            else:
                self.materials[name] = ''

        for mesh in root.findall('.//mesh'):
            name = mesh.attrib.get('name', '')
            file = mesh.attrib.get('file', '')
            if file.startswith('visual') or file.startswith('collision'): #HACK: the true path is hidden in some compiler attribute
                file = 'meshes/'+file
            mesh.attrib['file'] = file
            self.meshes[name] = mesh.attrib
    
    def add_node(self, node, f_parent, path, level):
        if 'file' in node.attrib:
            file = node.attrib['file']
            node.attrib['file'] = os.path.join(path, file)

        print('|'+level*'  ', node.tag, node.attrib)

        if node.tag == 'include':
            file = node.attrib['file'] 
            tree = ET.parse(file)
            root = tree.getroot()
            path, _ = os.path.split(file)
            self.load_assets(root, path)
            self.add_node(root, f_parent, path, level+1)

        f_body = None

        if node.tag == 'body':
            f_body = self.add_body(node, f_parent)

        for child in node:
            if f_body is None:
                self.add_node(child, f_parent, path, level+1)
            else:
                self.add_node(child, f_body, path, level+1)

    def add_body(self, body, f_parent):
        self.bodyCount += 1
        body_name = body.attrib.get('name', f'body_{self.bodyCount}')

        # if self.C.getFrame(body_name, False):
        body_name = f'{body_name}_{self.bodyCount}'
        f_body = self.C.addFrame(body_name)
        f_body.setParent(f_parent)
        self.setRelativePose(f_body, body.attrib)

        for i, joint in enumerate(body.findall('./joint')):
            axis = joint.attrib.get('axis', None)
            limits = joint.attrib.get('range', None)
            joint_name = joint.attrib.get('name', f'{body_name}_joint{i*"_"}')
            f_origin = self.C.addFrame(f'{joint_name}_origin')
            f_origin.setParent(f_body)
            self.setRelativePose(f_origin, joint.attrib)
            f_origin.unLink()
            f_origin.setParent(f_parent, True)

            if axis:
                if axis in self.muj2rai_joint_map:
                    axis = self.muj2rai_joint_map[axis]
                else:
                    vec1 = np.array([0., 0., 1.])
                    vec2 = np.array(self.as_floats(axis))
                    quat = ry.Quaternion().setDiff(vec1, vec2).asArr()
                    f_origin.setRelativeQuaternion(quat)
                    axis = ry.JT.hingeZ
            else:
                axis = ry.JT.hingeZ

            if joint.attrib.get('type', 'hinge')=='slide':
                trans_map = {
                    ry.JT.hingeX: ry.JT.transX,
                    ry.JT.hingeY: ry.JT.transY,
                    ry.JT.hingeZ: ry.JT.transZ}
                axis = trans_map[axis]

            if not limits:
                limits = '-1 1'

            # if self.C.getFrame(joint_name, False):
            #     joint_name = f'{joint_name}_{self.bodyCount}'
            f_joint = self.C.addFrame(joint_name)
            f_joint.setParent(f_origin)
            f_joint.setJoint(axis, self.as_floats(limits))
            
            # relink body:
            f_parent = f_joint
            f_body.unLink()
            f_body.setParent(f_parent, True)

        for i, geom in enumerate(body.findall('./geom')):
            isColl = geom.attrib.get('contype', self.defaultConType)!='0' or 'coll' in geom.attrib.get('class','')
            if self.visualsOnly and isColl:
                continue

            f_shape = self.C.addFrame(f'{body_name}_shape{i}')
            f_shape.setParent(f_body)

            if 'mesh' in geom.attrib:
                mesh = geom.attrib.get('mesh', '')
                material_name = geom.attrib.get('material', '')
                texture_path = self.materials.get(material_name, None)
                meshfile = self.meshes[mesh]['file']
                scale = self.as_floats(self.meshes[mesh].get('scale', '1 1 1'))

                f_shape.setMeshFile(meshfile, scale[0])
                
                if texture_path:
                    if len(texture_path.split()) == 4:  # Is a color rgba
                        f_shape.setColor(self.as_floats(texture_path))

            elif 'type' in geom.attrib:
                size = self.as_floats(geom.attrib['size'])
                if geom.attrib['type']=='capsule':
                    if 'fromto' in geom.attrib:
                        fromto = self.as_floats(geom.attrib['fromto'])
                        a, b = np.array(fromto[:3]), np.array(fromto[3:])
                        l = np.linalg.norm(b-a)
                        q = ry.Quaternion().setDiff([0,0,1],(b-a)/l)
                        f_shape.setRelativePosition(0.5*(a+b))
                        f_shape.setRelativeQuaternion(q.asArr())
                        f_shape.setShape(ry.ST.capsule, [l, size[0]])
                    elif len(size)==2:
                        f_shape.setShape(ry.ST.capsule, [2.*size[1], size[0]])

                elif geom.attrib['type']=='cylinder':
                    if len(size)==2:
                        f_shape.setShape(ry.ST.cylinder, [2.*size[1], size[0]])

                elif geom.attrib['type']=='box':
                    assert len(size)==3
                    size = [2.*f for f in size]
                    f_shape.setShape(ry.ST.box, size)
                    if geom.attrib.get('material', None):
                        texture_path = self.materials[geom.attrib.get('material', None)]
                        if len(texture_path.split()) == 4:  # Is a color rgba
                            f_shape.setColor(self.as_floats(texture_path))
                        else:
                            print('applying to box:', texture_path)
                            #TODO incorperate <texrepeat> tag correctly
                            uv_coords = np.array([
                            [0, 0],  # vertex 0
                            [size[0], 0],  # vertex 1
                            [size[0], size[1]],  # vertex 2
                            [0, size[1]],  # vertex 3
                            [0, 0],  # vertex 0
                            [size[0], 0],  # vertex 1
                            [size[0], size[1]],  # vertex 2
                            [0, size[1]],  # vertex 3
                            ])
                            
                            f_shape.setTextureFile(texture_path, uv_coords)

                elif geom.attrib['type']=='sphere':
                    if len(size)==1:
                        f_shape.setShape(ry.ST.sphere, size)
                
            self.setRelativePose(f_shape, geom.attrib)

            if geom.attrib.get('rgba', None):
                if geom.attrib.get('material', None) is None:
                    f_shape.setColor(self.as_floats(geom.attrib['rgba']))

            elif isColl:
                f_shape.setColor([1,0,0,.2])
                
        return f_body

    def setRelativePose(self, f, attrib):
        pos = attrib.get('pos', None)
        if pos:
            f.setRelativePosition(self.as_floats(pos))
        
        quat = attrib.get('quat', None)
        if quat:
            f.setRelativeQuaternion(self.as_floats(quat))
        
        rpy = attrib.get('euler', None)
        if rpy:
            q = ry.Quaternion()
            q.setRollPitchYaw(self.as_floats(rpy))
            f.setRelativeQuaternion(q.asArr())
