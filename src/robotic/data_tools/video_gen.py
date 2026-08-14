import numpy as np
from PIL import Image
from subprocess import Popen, PIPE

class VideoGenerator():
    def __init__(self, input_framerate=50, output_framerate=30, filename='tmp.mp4'):
        self.p = Popen(['ffmpeg', '-y', '-loglevel', 'warning', '-f', 'image2pipe',
                        '-vcodec', 'ppm', '-r', str(input_framerate), '-i', '-', #input
                        '-vcodec', 'libx264', '-r', str(output_framerate), '-pix_fmt', 'yuv420p', filename], #output
                       stdin=PIPE)

    def __del__(self):
        self.p.stdin.close()
        self.p.wait()

    def add(self, rgb: np.array, multiple=1):
        if isinstance(rgb, list):
            for slice in rgb:
                self.add(slice, multiple=1)
        else:
            assert rgb.ndim==3
            img = Image.fromarray(rgb, 'RGB')
            for _ in range(int(multiple)):
                img.save(self.p.stdin, 'PPM')
        