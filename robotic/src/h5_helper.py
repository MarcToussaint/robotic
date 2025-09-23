#!/usr/bin/env python3

import argparse
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
        self.fil = h5py.File(filename, 'r')

    def print_attrs(self, name, obj):
        if isinstance(obj, h5py.Dataset):
            print('   ', obj.name, obj.shape, obj.dtype, f'{obj.size*obj.dtype.itemsize/1024:.2f}kB')
            if obj.dtype=='int8':
                str = ''.join([chr(x) for x in obj[()]])
                print('       ', str)
            elif obj.size<20:
                print('       ', obj[()])
        else:
            print('---', obj.name)

    def print_info(self):
        self.fil.visititems(self.print_attrs)

    def read(self, name):
        return self.fil[name][()]

    def read_dict(self, name):
        obj = self.fil[name]
        assert obj.dtype=='int8'
        str = ''.join([chr(x) for x in obj[()]])
        d = ast.literal_eval(str)
        return d


