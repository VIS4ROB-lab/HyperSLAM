import os
import shutil
import subprocess
import sys

from typing import List
from datetime import datetime
from argparse import ArgumentParser
from conversions import convert_hyper_to_tum_format


class Sequence:
    def __init__(self, name: str, source: str, reference: str, offset: float = 0):
        self.name = name
        self.source = source
        self.reference = reference
        self.offset = offset

    def run(self, binary: os.path, settings: os.path, d_output: os.path):
        # Run the shell script.
        command = ["sh run.sh", binary, settings, self.source, d_output]
        subprocess.run(" ".join(command), shell=True)

        # Convert estimation file.
        p_hyper_estimation = os.path.join(d_output, "estimation.hyper")
        p_tum_estimation = os.path.join(d_output, "estimation.tum")
        convert_hyper_to_tum_format(p_hyper_estimation, p_tum_estimation)

        # Evaluate estimation.
        # TODO: Recover and pass on start and end times
        ape_rotation_command = \
            ["evo_ape", "tum", self.reference, p_tum_estimation,
             "-a -r angle_deg --plot_mode xy --silent",
             "--save_plot", os.path.join(d_output, "ape_rotation"),
             "--save_results", os.path.join(d_output, "ape_rotation_results.zip")]
        os.system(" ".join(ape_rotation_command))

        ape_translation_command = \
            ["evo_ape", "tum", self.reference, p_tum_estimation,
             "-a -r trans_part --plot_mode xy --silent",
             "--save_plot", os.path.join(d_output, "ape_translation"),
             "--save_results", os.path.join(d_output, "ape_translation_results.zip")]
        os.system(" ".join(ape_translation_command))

        rpe_rotation_command = \
            ["evo_rpe", "tum", self.reference, p_tum_estimation,
             "-a -r angle_deg --plot_mode xy --silent",
             "--save_plot", os.path.join(d_output, "rpe_rotation"),
             "--save_results", os.path.join(d_output, "rpe_rotation_results.zip")]
        os.system(" ".join(rpe_rotation_command))

        rpe_translation_command = \
            ["evo_rpe", "tum", self.reference, p_tum_estimation,
             "-a -r trans_part --plot_mode xy --silent",
             "--save_plot", os.path.join(d_output, "rpe_translation"),
             "--save_results", os.path.join(d_output, "rpe_translation_results.zip")]
        os.system(" ".join(rpe_translation_command))


class Dataset:
    def __init__(self, name: str, sequences: List[Sequence], d_setups: str):
        self.name = name
        self.sequences = sequences
        self.d_setups = d_setups

    def run(self, binary: str, setup: str, d_output: str):
        # Run sequences.
        print(f"\n### Started all sequences of {self.name} dataset in {setup} setup. >>>\n")
        for sequence in self.sequences:
            self.runSequence(binary, setup, d_output, sequence)
        print(f"\n<<< Finished all sequences of {self.name} dataset in {setup} setup. ###\n")

    def runSequence(self, binary: str, setup: str, d_output: str, sequence: Sequence):
        # Retrieve settings.
        settings_name = "settings.yaml"
        d_setup = os.path.join(self.d_setups, setup)
        if os.path.isfile(os.path.join(d_setup, settings_name)):
            sequence_settings = os.path.join(d_setup, settings_name)
        elif os.path.isdir(os.path.join(d_setup, sequence.name, settings_name)):
            sequence_settings = os.path.join(d_setup, sequence.name, settings_name)
        else:
            sys.exit(f"Settings do not exist.")

        # Create directories.
        d_sequence_output = os.path.join(d_output, self.name, setup, sequence.name)
        os.makedirs(d_sequence_output)

        # Copy settings.
        shutil.copy2(sequence_settings, os.path.join(d_sequence_output, "settings.yaml"))

        # Run sequence.
        print(f"\n### Started {sequence.name} sequence of {self.name} dataset in {setup} setup. >>>\n")
        sequence.run(binary, sequence_settings, d_sequence_output)
        print(f"\n<<< Finished {sequence.name} sequence of {self.name} dataset in {setup} setup. ###\n")


def run(datasets, binary: str, setup: str, d_output: str, s_datasets: List[str] = None, s_sequences: List[str] = None):
    if s_datasets:
        for s_dataset in s_datasets:
            dataset = next((dataset for dataset in datasets if dataset.name == s_dataset), None)
            if s_sequences:
                for s_sequence in s_sequences:
                    # Run single sequence.
                    sequence = next((sequence for sequence in dataset.sequences if sequence.name == s_sequence), None)
                    dataset.runSequence(binary, setup, d_output, sequence)
            else:
                # Run single dataset.
                dataset.run(binary, setup, d_output)
    else:
        # Run all datasets.
        print(f"\n### Started all datasets in {setup} setup. >>>\n")
        for dataset in datasets:
            dataset.run(binary, setup, d_output)
        print(f"\n<<< Finished all datasets in {setup} setup. ###\n")


def main():
    # Parse input arguments.
    parser = ArgumentParser()
    parser.add_argument("setups", type=str, nargs="+", help=f"Setups to evaluate.")
    parser.add_argument("--binary", type=str, help=f"Binary directory. Default directory if none is provided.")
    parser.add_argument("--sources", type=str, help=f"Source directory. Default directory if none is provided.")
    parser.add_argument("--datasets", type=str, nargs="*", help=f"Datasets to evaluate, or all if none is provided.")
    parser.add_argument("--sequences", type=str, nargs="*", help=f"Sequences to evaluate, or all if none is provided.")
    parser.add_argument("--output", type=str, help=f"Output directory. Default directory if none is provided.")
    args = parser.parse_args()

    # Define paths.
    p_binary = os.path.join("bin", "HyperSLAM")
    p_datasets = os.path.join("resources", "datasets")
    p_inputs = "rosbags"
    p_setups = "setups"
    p_sequences = "sequences"
    p_output = "output"

    # Evaluate paths.
    d_cwd = os.getcwd()
    d_hyper = os.path.dirname(d_cwd)
    d_datasets = args.sources if args.sources else os.path.join(d_hyper, p_datasets)
    d_output = args.output if args.output else os.path.join(d_cwd, p_output, datetime.now().strftime("%Y%m%d_%H%M%S"))

    datasets: List[Dataset] = []
    for dataset_name in sorted(os.listdir(d_datasets)):
        # Evaluate paths.
        d_dataset = os.path.join(d_datasets, dataset_name)
        d_dataset_sources = os.path.join(d_dataset, p_inputs)
        d_dataset_setups = os.path.join(d_dataset, p_setups)
        d_dataset_references = os.path.join(d_dataset, p_sequences)

        if os.path.isdir(d_dataset):
            sequences: List[Sequence] = []
            for sequence_file in sorted(os.listdir(d_dataset_sources)):
                sequence_name = os.path.splitext(sequence_file)[0]
                sequence_source = os.path.join(d_dataset_sources, sequence_name + ".bag")
                sequence_reference = os.path.join(d_dataset_references, sequence_name + ".txt")
                sequence = Sequence(sequence_name, sequence_source, sequence_reference)
                sequences.append(sequence)
            dataset = Dataset(dataset_name, sequences, d_dataset_setups)
            datasets.append(dataset)

    # Retrieve binary.
    binary = args.binary if args.binary else os.path.join(d_hyper, p_binary)

    # Iterate over setups.
    for setup in args.setups:
        run(datasets, binary, setup, d_output, args.datasets, args.sequences)


if __name__ == "__main__":
    main()
