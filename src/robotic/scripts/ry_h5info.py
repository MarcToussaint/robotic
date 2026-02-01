#!/usr/bin/env python3

import argparse
import h5py
from robotic.src.h5_helper import *

parser = argparse.ArgumentParser(description='h5-file info')

parser.add_argument('FILE', type=str,
                    help='h5-file name')

def main():
    args = parser.parse_args()

    print('=== file', args.FILE)
    try:
        h5 = H5Reader(args.FILE)
        h5.print_info()
    except KeyboardInterrupt:
        sys.exit(1)

if __name__ == "__main__":
    main()
