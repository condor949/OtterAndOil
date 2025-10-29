# OtterAndOil

The project simulates the movement of autonomous vessels and ground robots in a field with a given intensity (as a comprehensible example, imagine an oil spill). The code allows you to set space parameters, choose the type of vehicles and controller, and then run the simulation with visualization of the results.

## Installation

```bash
pip install -r requirements.txt
```

## Running the simulation

```bash
python main.py -c config.json
```

You can point to any configuration file by passing its path via `-c/--config-file`.
Relative paths are resolved from the repository root, so both
`python main.py -c configs/experiment.json` and `python main.py` (which uses the
default `config.json`) work.

*LaTeX is used when rendering plots, so a TeX distribution may be required for a full run. If you do not want to install LaTeX packages, set `use_latex` to `false` in your configuration to render labels using plain text instead.*

## How it works

The `main.py` script reads settings from the configuration file and creates the required objects: vehicles, space and controller. Controller instances are created and coordinated by `ControllerManager`, which can parallelize the numerical simulation phase across multiple controllers and offload track rendering to worker processes when animations are enabled.

For each series of runs the `DataStorage` class creates a timestamped directory inside the directory specified by `cache_dir` (defaults to `data`). When launched with `config.json`, the `data/expt_gaussian_s1_<time>` directory will contain:

- `gaussian_<time>.png` — map of the space intensity;
- `sigmas_v0_<time>.png`, `cumulative_<time>.png`, `intensity_<time>.png` — controller parameter plots;
- `track_<time>.png` and `track_<time>.gif` — agent trajectories.

## Project structure

- **controllers** – control algorithms and their infrastructure:
  - `BaseController` — basic interface for computing control actions and storing simulation settings;
  - `IntensityBasedController` — individual controller using Matveev's law and field intensity data to switch propellers ([doi:10.1109/TAC.2023.3284595](https://doi.org/10.1109/TAC.2023.3284595));
  - `IntensityAndLinearVelocityController` — nonlinear individual controller that couples intensity measurements with linear velocity regulation;
  - `IntensityAndLinearVelocityPIDController` — nonlinear controller with an additional PID loop for the linear velocity channel;
  - `SwarmController` — controller for managing a swarm of robots;
  - `manager.py` — orchestrates controller instantiation, simulation and result aggregation;
  - `plotter.py`, `plot_jobs.py` — generation of plots and asynchronous track rendering jobs.
- **vehicles** – vehicle models:
  - `Vehicle` — base class with common parameters such as starting point and dynamic methods;
  - `Dubins` — simplified model of a wheeled robot with wheel angular velocity control and chassis geometry parameters;
  - `Otter` — detailed model of the Otter USV catamaran with hull parameters and heading control system adapted from [PythonVehicleSimulator](https://github.com/cybergalactic/PythonVehicleSimulator/blob/master/src/python_vehicle_simulator/vehicles/otter.py).
- **spaces** – description of the exploration space:
  - `BaseSpace` — infrastructure for storing peaks, coordinate shifts and building the intensity surface;
  - `Gaussian3DSpace` — generation of Gaussian peaks of a given shape;
  - `Parabolic3DSpace` — parabolic peak shapes with negative values cut off.
- **tools** – auxiliary utilities:
  - `dataStorage` — creation of timestamped folders, configuration helpers and filesystem utilities;
  - `random_generators` — generators of random starting points, colors and other utilities;
  - `common` — functions for automatically calling visualization or saving methods.
- **lib** – library with dynamics functions and simultaneous simulation (`gnc.py`, `simultaneousLoop.py`, etc.).
- **space-genereator.py** – PyQt5 graphical tool for interactive construction of peak files.

## Configuration files

### `config.json`
The main launch configuration file. Fields:

| Field | Description |
|------|-------------|
| `clean_cache` | remove cached results in `data/` before launch (does not affect custom `cache_dir` values) |
| `big_picture` | render large images (requires more memory) |
| `not_animated` | disable track animation |
| `store_raw` | save raw simulation data |
| `separating_plots` | create separate plots for agents |
| `store_plot` | save images instead of showing on screen |
| `isometric` | isometric view of the track |
| `dynamic_error_max` | enable adaptive normalization threshold for controller error metrics |
| `for_publication` | adjust plot styles for publication-ready output |
| `plot_config` | path to a JSON file describing plot axes, titles and legends |
| `use_latex` | render plot labels via LaTeX (`true`) or plain text (`false`) |
| `error_max_cap` | upper bound for the normalized error threshold |
| `eps` | initial error tolerance used when `dynamic_error_max` is enabled |
| `smoothing` | exponential smoothing factor for the adaptive error threshold |
| `grid_size` | spatial grid resolution |
| `axis_abs_max` | half-length of the space axes |
| `isolines` | number of isolines on the surface |
| `peaks_filename` | file describing intensity peaks |
| `cache_dir` | directory to save results |
| `peak_type` | space type (`gaussian` or `parabolic`) |
| `controller_vehicle_pairs` | array describing controller-to-vehicle assignments (see below) |
| `shift_vehicle` | shift of all starting points |
| `shift_xyz` | shift of the entire space |
| `target_isoline` | level of the target isoline |
| `sim_time_sec` | simulation time in seconds |
| `sample_time` | discretization step |
| `cycles` | number of experiment repetitions |
| `radius` | radius for generating random start points |
| `FPS` | frame rate when saving animation |
| `V_current` | speed of the current medium |
| `beta_current` | direction of the current |
| `log_level` | logging verbosity (`INFO`, `DEBUG`, etc.) |

All fields can be seen in the original `config.json` file. The same set of parameters is implemented in the [`Arguments`](tools/dataStorage.py) class used for configuration serialization and for storing the effective configuration alongside the results.

#### `controller_vehicle_pairs`

To run several controllers simultaneously — for example, one controller per vehicle — describe the assignments in the `controller_vehicle_pairs` array. Each object has the following fields:

| Field | Description |
|-------|-------------|
| `controller_type` | name of the controller registered in [`controllers/__init__.py`](controllers/__init__.py) |
| `vehicle_type` | vehicle type handled by the controller |
| `vehicles` | number of vehicles of the specified type handled by the controller |
| `start_points` *(optional)* | list of starting points (must match the `vehicles` count if provided) |

If `start_points` is omitted, each vehicle starts at `[0, 0]`.

The total number of vehicles is derived automatically from this array; no separate `vehicles` field is required.

### `peaks_.json`
Defines the set of intensity peaks for the space. Each object contains the center coordinates (`x0`, `y0`), amplitude and width parameters `sigma_x` and `sigma_y`.

### `plot_config.json`
Controls axis selections, labels and legends for the generated plots. Keys correspond to plot names (e.g. `intensity`, `error`, `error_avg`), and each entry specifies which controller attributes to use for the X/Y axes and how to label the resulting chart.

## Auxiliary scripts
- `space-genereator.py` — allows you to interactively create a peak file describing the field shape. The result is saved in `peaks_*.json` format.

## License

The project is distributed under the MIT License. See `LICENSE` for the full text.
