#!/usr/bin/env python3

import argparse
import h5py

parser = argparse.ArgumentParser(description='h5-file info')

parser.add_argument('FILE', type=str,
                    help='h5-file name')

def print_attrs(name, obj):
    if isinstance(obj, h5py.Dataset):
        print('   ', name, obj.name, obj.shape, obj.dtype)
    else:
        print('---', name)

def main():
    args = parser.parse_args()

    print('=== file', args.FILE)
    try:
        with h5py.File(args.FILE, 'r') as fil:
            fil.visititems(print_attrs)
    except KeyboardInterrupt:
        sys.exit(1)

if __name__ == "__main__":
    main()
