import subprocess
import os
import yaml
from pathlib import PurePath

class DataInstaller:
    data_home = '$USER@hal-9000.lis.tu-berlin.de:/home/data/'

    def __init__(self, source, dest_path = './', dry_run=False):
        self.data_home = os.path.expandvars(self.data_home)
        self.data_dest = dest_path
        self.rsync = 'rsync -vrlptzP --update --mkpath --exclude=z.* --exclude=ex_*'.split()
        if dry_run:
            self.rsync += ['--dry-run']

        if 'yml' in source or 'yaml' in source:
            # this is a manifest file -- load it
            self.data_src = str(PurePath(source).parent)+'/'
            manifest_name = str(PurePath(source).name)
            print('--- base path:', self.data_src)
            cmd = self.rsync + [self.data_home+source, self.data_dest]
            print('--- loading manifest:', cmd)
            subprocess.run(cmd)

            with open(manifest_name, 'r', encoding='utf-8') as fil:
                self.manifest = yaml.safe_load(fil)
        else:
            # this is just a path
            self.data_src = source

        # print('--- manifest:\n', self.manifest)

    def pull(self):
        cmd = self.rsync + [self.data_home+self.data_src, self.data_dest]
        print('--- loading full folder:', cmd)
        subprocess.run(cmd)

    def push(self):
        cmd = self.rsync + ['--exclude=tmp'] + [self.data_dest, self.data_home+self.data_src]
        print('--- pushing full folder:', cmd)
        subprocess.run(cmd)

    def install(self, start=0, stop=-1):
        datasets = self.manifest['datasets']

        # write files to be copied into file
        with open('z.files', 'w', encoding='utf-8') as fil:
            for d in datasets[start:stop]:
                fil.write(str(PurePath(d).parent)+'/\n')
                fil.write(d+'\n')
            fil.write('- *\n')

        # load them
        cmd = self.rsync + ['--include-from=z.files', self.data_home+self.data_src, self.data_dest]
        print('--- loading datasets:', cmd)
        subprocess.run(cmd)
