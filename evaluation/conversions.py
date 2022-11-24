import numpy as np
from argparse import ArgumentParser


def convert_hyper_to_tum_format(input, output):
    data = np.loadtxt(input, delimiter=",")
    print(len(data))
    data = data[:, [0, 5, 6, 7, 1, 2, 3, 4]]
    np.savetxt(output, data, fmt='%.20e')


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("input", help="Input file.")
    parser.add_argument("output", help="Output file.")
    args = parser.parse_args()
    convert_hyper_to_tum_format(args.input, args.output)
