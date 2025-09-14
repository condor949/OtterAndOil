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

*LaTeX is used when rendering plots, so a TeX distribution may be required for a full run. If you do not want to install LaTeX packages, set `use_latex` to `false` in your configuration to render labels using plain text instead.*

## How it works

The `main.py` script reads settings from the configuration file and creates the required objects: vehicles, space and controller. For each series of runs the `DataStorage` class creates a timestamped directory inside `data` where all images are saved. When launched with `config.json`, the `data/expt_gaussian_s1_<time>` directory will contain:

- `gaussian_<time>.png` — map of the space intensity;
- `sigmas_v0_<time>.png`, `cumulative_<time>.png`, `intensity_<time>.png` — controller parameter plots;
- `track_<time>.png` and `track_<time>.gif` — agent trajectories.

## Project structure

- **controllers** – control algorithms:
  - `BaseController` — basic interface for computing control actions and storing simulation settings;
  - `IntensityBasedController` — individual controller using Matveev's law and field intensity data to switch propellers ([doi:10.1109/TAC.2023.3284595](https://doi.org/10.1109/TAC.2023.3284595));
  - `SwarmController` — controller for managing a swarm of robots.
- **vehicles** – vehicle models:
  - `Vehicle` — base class with common parameters such as starting point and dynamic methods;
  - `Dubins` — simplified model of a wheeled robot with wheel angular velocity control and chassis geometry parameters;
  - `Otter` — detailed model of the Otter USV catamaran with hull parameters and heading control system adapted from [PythonVehicleSimulator](https://github.com/cybergalactic/PythonVehicleSimulator/blob/master/src/python_vehicle_simulator/vehicles/otter.py).
- **spaces** – description of the exploration space:
  - `BaseSpace` — infrastructure for storing peaks, coordinate shifts and building the intensity surface;
  - `Gaussian3DSpace` — generation of Gaussian peaks of a given shape;
  - `Parabolic3DSpace` — parabolic peak shapes with negative values cut off.
- **tools** – auxiliary utilities:
  - `dataStorage` — creation of timestamped folders and saving configuration and results;
  - `random_generators` — generators of random starting points, colors and other utilities;
  - `common` — functions for automatically calling visualization or saving methods.
- **lib** – library with dynamics functions and simultaneous simulation (`gnc.py`, `simultaneousLoop.py`, etc.).
- **space-genereator.py** – PyQt5 graphical tool for interactive construction of peak files.

## Configuration files

### `config.json`
The main launch configuration file. Fields:

| Field | Description |
|------|-------------|
| `clean_cache` | remove old data before launch |
| `big_picture` | render large images (requires more memory) |
| `not_animated` | disable track animation |
| `store_raw` | save raw simulation data |
| `separating_plots` | create separate plots for agents |
| `store_plot` | save images instead of showing on screen |
| `isometric` | isometric view of the track |
| `grid_size` | spatial grid resolution |
| `axis_abs_max` | half-length of the space axes |
| `isolines` | number of isolines on the surface |
| `peaks_filename` | file describing intensity peaks |
| `cache_dir` | directory to save results |
| `peak_type` | space type (`gaussian` or `parabolic`) |
| `use_latex` | render plot labels via LaTeX (`true`) or plain text (`false`) |
| `controller_type` | controller in use |
| `vehicle_types` | list of vehicle types |
| `start_points` | starting coordinates of agents |
| `shift_vehicle` | shift of all starting points |
| `shift_xyz` | shift of the entire space |
| `target_isoline` | level of the target isoline |
| `sim_time_sec` | simulation time in seconds |
| `sample_time` | discretization step |
| `cycles` | number of experiment repetitions |
| `radius` | radius for generating random start points |
| `vehicles` | number of agents of each type |
| `FPS` | frame rate when saving animation |
| `V_current` | speed of the current medium |
| `beta_current` | direction of the current |

All fields can be seen in the original `config.json` file. The same set of parameters is implemented in the [`Arguments`](tools/dataStorage.py) class used for configuration serialization.

### `peaks_.json`
Defines the set of intensity peaks for the space. Each object contains the center coordinates (`x0`, `y0`), amplitude and width parameters `sigma_x` and `sigma_y`.

## Auxiliary scripts
- `space-genereator.py` — allows you to interactively create a peak file describing the field shape. The result is saved in `peaks_*.json` format.

## License

The project is distributed under the MIT License. See `LICENSE` for the full text.
