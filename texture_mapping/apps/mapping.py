import argparse
import os.path as osp
import subprocess

from PIL import Image

import texture_mapping


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--input', type=str,
                        help='Input ply filename', required=True)
    parser.add_argument('--out', '-o', type=str,
                        help='Output obj file name. This file is created at '
                        'the same dir of input ply file.',
                        required=True)
    parser.add_argument('--config-dir', '-c', type=str,
                        help='Config directory', required=True)
    args = parser.parse_args()

    occluded_file = osp.join(osp.dirname(args.input), 'occluded.jpg')
    if not osp.exists(occluded_file):
        white_img = Image.new('RGB', (256, 256), (255, 255, 255))
        white_img.save(occluded_file)

    subprocess.call(
        [texture_mapping.texture_mapping_executable,
         args.input,
         osp.join(osp.dirname(args.input), osp.basename(args.out)),
         args.config_dir])


if __name__ == '__main__':
    main()
