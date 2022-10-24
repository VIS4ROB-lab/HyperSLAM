# Evaluation

## Setup

The evaluation scripts are largely based on the Python language, hence, we recommend using a virtual environment:

    # Create virtual environment.
    cd HyperSLAM/evaluation
    python3 -m venv venv
    source venv/bin/activate

    # Install dependencies.
    pip install -r requirements.txt

    # Source ROS distribution.
    source /opt/ros/noetic/setup.sh

    # (Optional) Additional EVO configurations.
    evo_config set plot_seaborn_style "whitegrid"

The necessary input files for every dataset, sequence and setup must be provided in the `resources/datasets` directory,
which must follow the same layout as the existing datasets. Note that the sequences files (i.e. ground truth files) and
the setup files must be named in accordance with the name of the provided ROS input bag files.

## Evaluation

To evaluate all datasets in a certain setup, simply execute:

    # Evaluates all datasets in a stereo setup.
    python run.py stereo

    # Signature.
    python run.py setup --datasets <dataset_0> ... --sequences <sequence_0> ...

Additionally, it is possible to only evaluate a specific dataset with the `--datasets` option:

    # Evaluates the EuRoC dataset in a stereo setup.
    python run.py stereo --datasets euroc

The `--sequences` option allows for even further refinement:

    # Evaluates the MH_01_easy and MH_02_easy sequences of the EuRoC dataset in a stereo setup.
    python run.py stereo --datasets euroc --sequences MH_01_easy MH_02_easy

## Output

Output files are stored inside the `output` directory and labeled chronologically by default. In particular, the
generated output directories have the following layout:

    - <YYYYMMDD_HHMMSS>
        - <dataset_0>
            - <setup_0>
                - <sequence_0>
                    - settings.yaml
                    - estimation.hyper
                    - estimation.tum
                    - <###>_map.png
                    - <###>_raw.png
                    - <###>_results.zip
                - ...
                - <sequence_i>
            - ...
            - <setup_j>
        - ...
        - <dataset_k>

Above, the `estimation.hyper` file is the estimate provided by *Hyper*SLAM, whereas the `estimation.tum` file has been
converted to TUM style to make it compatible with the EVO evaluation tool. In addition, we also copy the employed
`settings.yaml` file for later usage and produce several plots for each evaluated sequence. The `<###>_results.zip`
folders contain information which is required for further comparisons/analysis (see below).

## Comparison

To compare multiple result folders with the above layout, simply run:

    python comparison.py --comparisons <folder_0> <folder_1> ... --labels <label_0> <label_1> ...

You can compare arbitrary many results, however, the number of arguments provided to the `--comparisons` option must
match the ones provided to the `--labels` option. It needs to be pointed out that only datasets and sequences common to
ALL configurations are compared.
