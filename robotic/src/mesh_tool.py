import os
import numpy as np
#import matplotlib.pyplot as plt
import trimesh
import pymeshlab as meshlab
import base64
import yaml
import h5py
import matplotlib.pyplot as plt
from PIL import Image

def write_arr(X, fil, type='float32'):
    data = (type, list(X.shape), )
    fil.write(f'[ "{type}", {list(X.shape)}, "')
    fil.write(base64.b64encode(X.astype(type)).decode('utf-8'))
    fil.write('" ]')

def conv_tuple_arr(data_tuple):
    X = np.frombuffer(base64.decodebytes(bytearray(data_tuple[2].encode('utf-8'))), dtype=data_tuple[0])
    X = X.reshape(data_tuple[1])
    return X

class MeshTool():
    def __init__(self, file):
        self.failed = False
        self.inertiaIsDiagonal = False
        if file[-3:]==".h5":
            self.load_h5(file)
        else:
            self.load(file)
            self.file = file

    def load(self, file):
        print('=== loading mesh: ', file)
        self.filebase = os.path.splitext(file)[0]
        self.tmesh = trimesh.load(file, force='mesh')
        try:
            scene_or_mesh = trimesh.load(file, force='mesh')
        except Exception as e:
            print(e)
            print('  load failed:', file)
            return None

        if isinstance(scene_or_mesh, trimesh.Scene):
            if len(scene_or_mesh.geometry) == 0:
                self.tmesh = None
                self.failed = True
            else:
                # we lose texture information here
                self.tmesh = trimesh.util.concatenate(
                    tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                        for g in scene_or_mesh.geometry.values()))
        else:
            assert(isinstance(scene_or_mesh, trimesh.Trimesh))
            self.tmesh = scene_or_mesh

    def view(self):
        self.tmesh.show(resolution=(300,300))

    def report(self):
        print('  #V:', self.tmesh.vertices.shape,
              '  #T:', self.tmesh.faces.shape, end='')
        if hasattr(self.tmesh.visual, 'vertex_colors'):
            print('  #C:', self.tmesh.visual.vertex_colors.shape, end='')
        if hasattr(self.tmesh.visual, 'uv') and  hasattr(self.tmesh.visual.uv, 'shape'):
            print('  #uv:', self.tmesh.visual.uv.shape, end='')
        print(f'  [{self.filebase}]')

    def autoScale(self):
        scale = 1
        #if self.mesh.scale > 10000:
        #    scale = 1e-5
        #elif self.mesh.scale > 1000:
        #    scale = 1e-4
        if self.tmesh.scale > 200:
            scale = 1e-3
        elif self.tmesh.scale > 20:
            scale = 1e-2
        elif self.tmesh.scale > 2:
            scale = 1e-1
        #translate = -mesh.centroid
        translate = -.5*(self.tmesh.bounds[0]+self.tmesh.bounds[1])
        matrix = np.eye(4)
        matrix[0:3, 3] = translate
        matrix[0:3, 0:4] *= scale
        self.tmesh.apply_transform(matrix)

    def transformInertia(self, verbose=False):
        print("prior COM" , self.tmesh.center_mass)
        print("prior inertia\n" , self.tmesh.moment_inertia)

        U, D, V = np.linalg.svd(self.tmesh.moment_inertia)

        # Ensure proper rotation  
        if np.linalg.det(V) < 0:
            V[:, -1] *= -1

        matrix = np.eye(4)
        matrix[0:3, 3] = V @ (-self.tmesh.center_mass)  # Move center of mass to origin
        matrix[0:3, 0:3] = V  # Rotate the mesh to align with principal axes

        self.tmesh.apply_transform(matrix)

        if verbose:
            print("COM after transformation:" , self.tmesh.center_mass)
            print("Inertia after transfromation:\n" , self.tmesh.moment_inertia)
        self.inertiaIsDiagonal = True

        return np.linalg.inv(matrix)

    def repair_trimesh(self, mergeTolerance=1e-6):
        print('   - trimesh repair')
        try:
            trimesh.constants.tol.merge = mergeTolerance
            self.tmesh.process(validate=True)
            trimesh.repair.fill_holes(self.tmesh)
            trimesh.repair.fix_inversion(self.tmesh, multibody=True)
        except Exception as e:
            print('  --- repair failed ---', e)
            print('  --- this might be a trimesh bug: change order within mesh.process method')
            self.failed = True
            exit(0) # this might be a trimesh bug: change order within mesh.process method

        # if self.mesh.visual != None:
            # self.mesh.visual.uv = np.array([[0,0]])

    def repair_meshlab(self, merge_threshold=1e-4):
        print('   - meshlab repair')
        mlmesh = meshlab.Mesh(vertex_matrix=self.tmesh.vertices, face_matrix=self.tmesh.faces)
        ms = meshlab.MeshSet()
        ms.add_mesh(mlmesh)
        ms.apply_filter('meshing_merge_close_vertices', threshold=meshlab.PureValue(merge_threshold))
        # ms.apply_filter('meshing_remove_t_vertices')
        # ms.apply_filter('meshing_repair_non_manifold_edges')
        # ms.apply_filter('meshing_close_holes', maxholesize=1000)
        # ms.apply_filter('meshing_decimation_quadric_edge_collapse', targetfacenum=0, targetperc=0.5)
        ms.apply_filter('meshing_repair_non_manifold_edges')
        ms.apply_filter('meshing_close_holes', maxholesize=1000)
        # ms.print_filter_list()
        # print(ms.filter_parameter_values('meshing_merge_close_vertices'))
        # print(ms.filter_parameter_values('meshing_remove_t_vertices'))
        # print(ms.filter_parameter_values('meshing_close_holes'))
        # print(ms.filter_parameter_values('meshing_decimation_quadric_edge_collapse'))
        # mlmesh.compact()

        mlmesh = ms.current_mesh()
        V = mlmesh.vertex_matrix()
        T = mlmesh.face_matrix()
        self.tmesh = trimesh.Trimesh(vertices=V, faces=T)

    def clear_visual(self):
        self.tmesh = trimesh.Trimesh(vertices=self.tmesh.vertices, faces=self.tmesh.faces)

    def texture2vertexColors(self):
        if hasattr(self.tmesh.visual, 'uv'):
            colors = self.tmesh.visual.material.to_color(self.tmesh.visual.uv)
            vis = trimesh.visual.ColorVisuals(mesh=self.tmesh, vertex_colors=colors)
            self.tmesh.visual = vis
        # else:
            # raise ValueError("Mesh does not have UV coordinates!")

    def export_ply(self, filename=None):
        if filename is None:
            filename = self.filebase+'-.ply'
        print('  exporting', filename)
        self.tmesh.export(filename)

    def export_scene(self, convex=False):
        with open(self.filebase+'.g', 'w', encoding='utf-8') as fil:
            if self.inertiaIsDiagonal:
                if convex:
                    fil.write(f'obj: {{ X:[0., 0., 1.], mesh_decomp: <{self.filebase}.h5>, mass: {self.tmesh.center_mass.tolist()}, inertia: {np.diagonal(self.tmesh.moment_inertia).tolist()} }}\n')
            else:
                fil.write(f'obj: {{ X:[0., 0., 12.], mass: {self.tmesh.center_mass.tolist()}, inertia: {self.tmesh.moment_inertia.reshape([9]).tolist()} }}\n')
            fil.write(f'obj_mesh (obj): {{ mesh: <{self.filebase}.h5> }}\n')
            #fil.write(f'obj_points (obj): {{ mesh_points: <{self.filebase}.h5>, color: [1 1 0], size: [2.] }}\n')
            return self.filebase+'.g'

    def createPoints(self):
        self.pts, faces = trimesh.sample.sample_surface(self.tmesh, 20000)
        self.normals = self.tmesh.face_normals[faces]
        #bary = trimesh.triangles.points_to_barycentric(self.mesh.triangles[faces], pts)
        #normals = trimesh.unitize((self.mesh.vertex_normals[self.mesh.faces[faces]] *
        #                          trimesh.unitize(bary).reshape((-1, 3, 1))).sum(axis=1))
    

    def createDecomposition(self):

        convex_hulls = self.tmesh.convex_decomposition()
        
        self.decomp_parts = [0]
        self.decomp_vertices = np.asarray(convex_hulls[0].vertices)
        self.decomp_faces = np.asarray(convex_hulls[0].faces)
        vert_len = len(convex_hulls[0].vertices) 
        for i, part in enumerate(convex_hulls[1:]):                

            self.decomp_vertices = np.concatenate([self.decomp_vertices, part.vertices]) 
            self.decomp_faces = np.concatenate([self.decomp_faces, part.faces + vert_len])  
            self.decomp_parts.append(len(part.vertices)+self.decomp_parts[i])

            vert_len+=len(part.vertices)
        
        self.decomp_parts = np.asarray(self.decomp_parts)
        self.decomp_colors = [0,255,255]  # fixed blue for now

    def createDecomposition_lowlevel(self):
        ### create decomposition
        ret = os.system('ry-meshTool.sh ' + self.filebase+'.mesh' + ' -decomp -hide -quiet'
                        ' && mv z.arr ' + self.filebase+'.decomp' )
        if ret>0:
            print('  --- decomposition failed --- return:', ret)
            self.failed=True
            return

        ### load decomposition
        with open(self.filebase+'.decomp', 'r') as fil:
            decomp = yaml.safe_load(fil)
        self.decomp_vertices = conv_tuple_arr(decomp['V'])
        self.decomp_faces = conv_tuple_arr(decomp['T'])
        self.decomp_colors = conv_tuple_arr(decomp['C'])
        self.decomp_parts = conv_tuple_arr(decomp['cvxParts'])

    def export_h5(self, filename=None, without_colors=False, inertia=False):
        if filename is None:
            filename = self.filebase+'.h5'
        print('  exporting', filename)
        with h5py.File(filename, 'w') as fil:
            fil.create_dataset('mesh/vertices', data=self.tmesh.vertices, dtype='float32')
            assert self.tmesh.faces.shape[1]==3, 'can only export triangle meshes'
            if(self.tmesh.vertices.shape[0]<65535):
                fil.create_dataset('mesh/faces', data=self.tmesh.faces, dtype='uint16')
            else:
                fil.create_dataset('mesh/faces', data=self.tmesh.faces, dtype='uint32')
            
            if hasattr(self.tmesh.visual, 'vertex_colors') and not without_colors:
                colors = np.asarray(self.tmesh.visual.vertex_colors)[:,0:3]
                fil.create_dataset('mesh/colors', data=colors, dtype='uint8')

            if hasattr(self.tmesh.visual, 'uv') and not without_colors:
                texCoords = self.tmesh.visual.uv.copy()
                texCoords[:, 1] = 1 - texCoords[:, 1]
                fil.create_dataset('mesh/textureCoords', data=texCoords, dtype='float32')

                if hasattr(self, 'textureFile'):
                    file = np.frombuffer(self.textureFile.encode(), dtype=np.int8)
                    file = np.append(file, [0])
                    fil.create_dataset('mesh/textureFile', data=file, dtype='int8')
                else:
                    if hasattr(self.tmesh.visual.material, 'baseColorTexture'):
                        img = self.tmesh.visual.material.baseColorTexture
                    else:
                        img = self.tmesh.visual.material.image
                    if img is not None:
                        texImg = 255.*np.asanyarray(img.convert("RGB"))
                        fil.create_dataset('mesh/textureImg', data=texImg, dtype='uint8')

            if hasattr(self, 'pts') and self.pts.shape[1]==3:
                fil.create_dataset('points/vertices', data=self.pts, dtype='float32')
                fil.create_dataset('points/normals', data=self.normals, dtype='float32')

            if hasattr(self, 'decomp_vertices') and self.decomp_vertices.shape[1]==3:
                fil.create_dataset('decomp/vertices', data=self.decomp_vertices, dtype='float32')
                assert self.decomp_faces.shape[0]<65535
                fil.create_dataset('decomp/faces', data=self.decomp_faces, dtype='uint16')
                if not without_colors:
                fil.create_dataset('decomp/colors', data=self.decomp_colors, dtype='uint8')
                assert self.decomp_parts.shape[ 0]<65535
                fil.create_dataset('decomp/parts', data=self.decomp_parts, dtype='uint16')

            if inertia:
                fil.create_dataset('inertia/mass', data=[self.tmesh.mass], dtype='float32')
                fil.create_dataset('inertia/com', data=self.tmesh.center_mass, dtype='float32')
                if self.inertiaIsDiagonal:
                    fil.create_dataset('inertia/tensor', data=np.diagonal(self.tmesh.moment_inertia), dtype='float32')
                else:
                    fil.create_dataset('inertia/tensor', data=self.tmesh.moment_inertia, dtype='float32')
            fil.create_dataset('geo/centroid', data=self.tmesh.centroid, dtype='float32')
            fil.create_dataset('geo/bounds', data=self.tmesh.bounds, dtype='float32')
        
    def load_h5(self, file):
        print('=== file: ', file)
        self.filebase = os.path.splitext(file)[0]
        path, _ = os.path.split(file)
        with h5py.File(file, 'r') as fil:
            V = fil['mesh/vertices'][()]
            T = fil['mesh/faces'][()]
            if 'mesh/colors' in fil:
                C = fil['mesh/colors'][()]
            else:
                C = None

            texture = None
            texImage = None
            if 'mesh/textureFile' in fil:
                self.textureFile = fil['mesh/textureFile'][()]
                self.textureFile = ''.join([chr(i) for i in self.textureFile])[:-1] #cut the last embedding zero
                print(self.textureFile)
                texImage = Image.open(os.path.join(path, self.textureFile))
                material = trimesh.visual.texture.SimpleMaterial(image=texImage)
            if 'mesh/textureImg' in fil:
                texImage = fil['mesh/textureImg'][()]
                material = trimesh.visual.texture.SimpleMaterial(image=texImage)
            if 'mesh/textureCoords' in fil and texImage is not None:
                uv = fil['mesh/textureCoords'][()]
                uv[:, 1] = 1 - uv[:, 1]
                texture = trimesh.visual.TextureVisuals(uv=uv, image=texImage)

            if texture is None:
                self.tmesh = trimesh.Trimesh(vertices=V, faces=T, vertex_colors=C, process=True)
            else:
                self.tmesh = trimesh.Trimesh(vertices=V, faces=T, visual=texture, material=material, process=True)

                # colors = self.mesh.visual.material.to_color(self.mesh.visual.uv)
                # colors = mat.to_color(uv)
                # vis = trimesh.visual.ColorVisuals(mesh=self.mesh, vertex_colors=colors)
                # self.mesh.visual = vis


    def apply_texture(self, texture_path):
        try:
            texture_image = Image.open(texture_path)
            texture = trimesh.visual.TextureVisuals(image=texture_image, uv=self.tmesh.visual.uv)
            self.tmesh.visual = texture
            
            # texture = np.array(texture_image).astype(float) / 255.0
            # if texture.shape[-1] == 3:
            #     alpha = np.ones((texture.shape[0], texture.shape[1], 1))
            #     texture = np.concatenate([texture, alpha], axis=-1)
            
            # if not hasattr(self.mesh.visual, 'uv'):
            #     raise ValueError("Mesh does not have UV coordinates!")
            
            # uv_coords = self.mesh.visual.uv
            # uv_coords_copy = uv_coords.copy()
            # uv_coords_copy[:, 1] = 1 - uv_coords_copy[:, 1]
            
            # pixel_coords = (uv_coords_copy * [texture.shape[1] - 1, texture.shape[0] - 1]).astype(int)
            # vertex_colors = texture[pixel_coords[:, 1], pixel_coords[:, 0]]
            
            # self.mesh.visual = trimesh.visual.ColorVisuals(mesh=self.mesh, vertex_colors=vertex_colors)
        except Exception as e:
            print(texture_path, "is not a valid texture path or could not be applied to the mesh.", e)


    # def apply_scaling(self, tri_obj, scale):
    #     if scale != 1.0:
    #         print(scale)
    #         scaling_mat = scale * np.eye(4)
    #         scaling_mat[-1, -1] = 1.0
    #         tri_obj.apply_transform(scaling_mat)


    # def export_mesh(self, tri_obj, meshfile, ply_out, transformInertia, cvxDecomp):
    #     tri_obj.export(ply_out)
    #     print(f"Converted {meshfile}")
    #     M = MeshHelper(ply_out)
    #     if transformInertia:
    #         self.pose = M.transformInertia()
    #     if cvxDecomp:
    #         M.createDecomposition()
    #     M.export_h5(meshfile[:-3]+'h5', inertia=False)


    # def obj2ply(self, ply_out: str, scale: float=1.0, texture_path: str="none", transformInertia = False, cvxDecomp = False) -> bool:
    #     tri_obj = self.mesh
    #     if hasattr(tri_obj.visual, 'to_color'):
    #         if texture_path == "none":
    #             tri_obj.visual = tri_obj.visual.to_color()
    #         else:
    #             self.apply_texture(tri_obj, texture_path)
            
    #     elif hasattr(tri_obj.visual, 'vertex_colors'):
    #         print("Mesh already has vertex colors.")
        
    #     else:
    #         print(f"Failed on {tri_obj}")
    #         return False
        
    #     self.apply_scaling(tri_obj, scale)
    #     self.export_mesh(tri_obj, self.file, ply_out, transformInertia, cvxDecomp)
        
    #     return True
    
def timeout(signum, frame):
    raise Exception("timeout handler")

def get_sdf(mesh, N=30):
    #scale_mesh(mesh, .1)
    # get bounds
    bounds = mesh.bounds.copy()
    size = bounds[1] - bounds[0]
    # enlarge by 10%
    bounds[0] = bounds[0] - .1 * size
    bounds[1] = bounds[1] + .1 * size
    size = bounds[1] - bounds[0]
    # decide on a voxel size
    voxelSize = 1. / (N * np.power(np.prod(size), -1. / 3))
    gridDim = (size / voxelSize).astype(int) + 2
    # change bounds[1] to be on the grid
    bounds[1] = bounds[0] + voxelSize * (gridDim - 1)
    print('  sdf voxel:', voxelSize, 'dim:', gridDim, 'mem:', np.prod(gridDim))
    # create grid
    x_range = np.linspace(bounds[0, 0], bounds[1, 0], num=gridDim[0])
    y_range = np.linspace(bounds[0, 1], bounds[1, 1], num=gridDim[1])
    z_range = np.linspace(bounds[0, 2], bounds[1, 2], num=gridDim[2])
    grid = np.stack(np.meshgrid(x_range, y_range, z_range, indexing='ij'), axis=-1)
    # get sdf
    #sdf = -mesh.nearest.signed_distance(grid).reshape(gridDim)
    sdf = np.empty(gridDim)
    for z in range(0, sdf.shape[2]):
        print('\r  slice', z, end=' ', flush=True)
        sdf[:,:,z] = -mesh.nearest.signed_distance(grid[:,:,z,:].reshape(-1, 3)).reshape(gridDim[:2])
    print('- done')
    #scale_mesh(mesh, 10.)
    #bounds *= 10.
    return [sdf, bounds]

def display_sdf(sdf):
    ax = plt.subplot()
    cax = plt.axes([0.85, 0.1, 0.075, 0.8])
    for z in range(0, sdf.shape[2]):
        print(z)
        print(sdf[:, :, z])
        im = ax.imshow(sdf[:, :, z])
        plt.colorbar(im, cax=cax)
        plt.pause(.2)


        
