#!/usr/bin/env python3

import h5py
import json
import ast

class H5Writer:
    def __init__(self, filename):
        self.fil = h5py.File(filename, 'w')

    def write(self, name, data, dtype = 'float64'):
        self.fil.create_dataset(name, data=data, dtype=dtype)

    def write_dict(self, name, data):
        self.write(name, bytearray(json.dumps(data), 'utf-8'), dtype='int8')

class H5Reader:
    def __init__(self, filename):
        self.filename = filename
        self.fil = h5py.File(filename, 'r')

    def print_attrs(self, name, obj):
        if isinstance(obj, h5py.Dataset):
            print('   ', obj.name, obj.shape, obj.dtype, f'{obj.size*obj.dtype.itemsize/1024:.2f}kB')
            if obj.dtype=='int8':
                str = ''.join([chr(x) for x in obj[()]])
                print('       ', str)
            elif obj.dtype=='object':
                total_size=0
                print('       ', end='')
                for i in range(obj.size):
                    o = obj[i]
                    if obj.size<20:
                        print(o.shape, end='')
                    total_size += o.size*o.dtype.itemsize
                print(obj[0].dtype, f'{total_size/1024:.2f}kB')
            elif obj.size<20:
                print('       ', obj[()])
        else:
            print('---', obj.name)

    def print_info(self):
        self.fil.visititems(self.print_attrs)

    def read(self, name):
        try:
            X = self.fil[name][()]
        except:
            raise Exception(f'cannot access field "{name}" in {self.filename} file')
        return X

    def read_dict(self, name):
        obj = self.fil[name]
        assert obj.dtype=='int8'
        str = ''.join([chr(x) for x in obj[()]])
        d = ast.literal_eval(str)
        return d

if __name__ == "__main__":
    filename = sys.argv[1]
    print('=== file', filename)
    try:
        h5 = H5Reader(filename)
        h5.print_info()
    except KeyboardInterrupt:
        sys.exit(1)

