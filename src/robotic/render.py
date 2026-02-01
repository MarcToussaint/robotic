import robotic as ry
import numpy as np
import nvisii

def quatMult(b, c):
    a = b.copy();
    if(c[0]!=1.):
        a[0] *= c[0];  a[1] *= c[0];  a[2] *= c[0];  a[3] *= c[0];
    if(c[1]!=0.):
        a[0] -= b[1]*c[1];  a[1] += b[0]*c[1];  a[2] += b[3]*c[1];  a[3] -= b[2]*c[1];
    if(c[2]!=0.):
        a[0] -= b[2]*c[2];  a[1] -= b[3]*c[2];  a[2] += b[0]*c[2];  a[3] += b[1]*c[2];
    if(c[3]!=0.):
        a[0] -= b[3]*c[3];  a[1] += b[2]*c[3];  a[2] -= b[1]*c[3];  a[3] += b[0]*c[3];
    return a;

def flipQuat(quat):
    w = quat[0]
    quat[0]=quat[1]
    quat[1]=quat[2]
    quat[2]=quat[3]
    quat[3]=w
    return quat

class NvisiiRenderer:
    def __init__(self, width, height, focalLength=1.):
        nvisii.initialize(headless = True, verbose = True)
        nvisii.enable_denoiser()

        self.width = width
        self.height = height

        self.camera = nvisii.entity.create(
            name = "camera",
            transform = nvisii.transform.create(name = "camera_transform"),
            camera = nvisii.camera.create_from_focal_length(
                name = "camera_camera", 
                focal_length =  focalLength * height,
                sensor_width = width,
                sensor_height = height
            )
        )

        nvisii.set_camera_entity(self.camera)

        nvisii.set_dome_light_intensity(.5)
        nvisii.set_dome_light_color([1,1,1])
        # nvisii.disable_dome_light_sampling()

    def __del__(self):
        print('-- shutting down Nvisii')
        nvisii.deinitialize()

    def addConfig(self, C):
        frames = C.getFrameNames()

        for name in frames:
            # print(name)
            f = C.getFrame(name)
            F = f.info()
            if 'shape' in F and F['shape'] in ['mesh', 'ssBox']:
                print('=== creating', name)
                # print(F)
                V = f.getMeshPoints()
                T = f.getMeshTriangles()
                col = F['color']
                if len(col) == 4:
                    continue
                pos = f.getPosition()
                quat = f.getQuaternion()

                if not 'temperature' in F:
                    if len(col) == 1:
                        col = [col[0], col[0], col[0]]
                    if col == None:
                        col = [.8, .5, .3]
                    obj = nvisii.entity.create(
                        name = name,
                        mesh = nvisii.mesh.create_from_data(name=name, positions=V.flatten(), indices=T.flatten()),
                        transform = nvisii.transform.create(name=name, position=pos, rotation=flipQuat(quat)),
                        material = nvisii.material.create(name)
                    )
                    obj.get_material().set_base_color(col)  
                    obj.get_material().set_roughness(0.7)   
                    obj.get_material().set_specular(.5)
                    obj.get_material().set_sheen(.5)

                else:
                    print('=== creating light', name)
                    light = nvisii.entity.create(
                        name = name,
                        mesh = nvisii.mesh.create_from_data(name=name, positions=V.flatten(), indices=T.flatten()),
                        transform = nvisii.transform.create(name=name, position=pos, rotation=flipQuat(quat)),
                        light = nvisii.light.create(f'{name}_light')
                    )
                    if len(col) == 1:
                        light.get_light().set_intensity(col[0])
                    else:
                        light.get_light().set_intensity(1.)
                    light.get_light().set_exposure(3)
                    # light.get_light().set_color(col)
                    light.get_light().set_temperature(F['temperature'])
                    # light.get_light().use_surface_area(True)


    def setCamera(self, C: ry.Config):
        X = C.view_pose()
        pos = X[:3]
        quat = X[3:]
        quat = quatMult(quat, [0,1,0,0])
        quat = quat / np.linalg.norm(quat)
        self.camera.get_transform().set_position(pos)
        self.camera.get_transform().set_rotation(flipQuat(quat))

    def render(self, samples=64, flip=True):
        print('rendering...')
        # nvisii.render_to_file(self.width, self.height, 50, '01_simple_scene.png')
        buffer = nvisii.render(self.width, self.height, samples)
        img = np.array(buffer, dtype='float32').reshape(self.height, self.width, -1)
        img = img[:,:,:3]
        rgb = np.clip(255.*img, 0, 255).astype(np.uint8)
        if flip:
            rgb = np.flip(rgb, axis=0)
        return rgb
    
    
