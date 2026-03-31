# Exact and Decomposed Formulations for a Realistic Single-Truck Loading Problem with Stacking Constraints

This repository contains the reference implementation for the paper *Exact and decomposed formulations for a realistic single-truck loading problem with stacking constraints*. It implements exact and decomposed optimization models for a realistic single-truck loading problem in which items must be selected, stacked, and placed on a truck floor while satisfying compatibility, orientation, unloading-order, support, and axle-weight constraints. The codebase is written in Julia and uses JuMP with Gurobi to solve the MILP formulations. The repository also includes benchmark instances and archived experiment outputs used for the paper.

Implemented model variants:

- `IT-0`: baseline integrated item formulation
- `IT`: strengthened integrated item formulation
- `IT-H`: integrated item formulation warm-started from heuristic solutions
- `ST`: two-phase stack-then-pack formulation

## Repository contents

Main files and directories:

- `perform.jl`: command-line entry point for running one instance with one algorithm
- `read.jl`: instance and warm-start CSV readers
- `write.jl`: output writers for trucks, stacks, items, and run summaries
- `it_model.jl`: integrated item model
- `it_h_model.jl`: warm-started integrated item model
- `st_model.jl`: stack-based two-phase model
- `stack_making.jl`: stack-generation heuristic and exact stack formation model
- `instances/`: benchmark instances stored as `.tar` bundles
- `results/`: archived outputs and logs for the reproduced experiments
- `heuristic/`: legacy external heuristic binaries and support files used historically to generate warm starts

## Requirements

To run the models, you need:

- Julia 1.12 or a compatible Julia 1.x release
- Gurobi 12.0 with a valid license
- Julia packages:
  - `CSV`
  - `DataFrames`
  - `Tar`
  - `Dates`
  - `SHA`
  - `JuMP`
  - `Gurobi`
- optionally, `GRB_LICENSE_FILE` pointing to your Gurobi license file

The exact models (`it0`, `it`, `ith`, `st`) run natively in Julia through JuMP/Gurobi.

The bundled executable under `heuristic/` is Windows-specific and should be treated as optional legacy material. It is not required to reproduce the published `IT-H` experiments in this repository, because the warm-start CSV files already used by `IT-H` are stored under `results/heur_warmstart_rand_iter100/`.

## Input data format

Each instance is expected to be a `.tar` archive containing exactly these three files:

- `input_items.csv`
- `input_parameters.csv`
- `input_trucks.csv`

The parser expects the ROADEF-style semicolon-delimited CSV format used throughout this repository.

## Running the code

### Quick start

Run the stack-based model on one bundled instance:

```bash
julia perform.jl instances/BY5_Camiones56.tar st 60 out
```

### Command-line interface

The main entry point is:

```bash
julia perform.jl <instance_tar> <alg> <max_cpu> <out_dir> [warmstarts_dir]
```

Supported values for `<alg>`:

- `it0`: baseline integrated model
- `it`: integrated model with bounds and symmetry breaking
- `ith`: integrated model with warm start loaded from CSV results
- `st`: two-phase stack-then-pack model


Notes:

- `<max_cpu>` is the time limit passed to the main run, in seconds.
- The script performs a short preliminary run before the main solve in order to trigger model-family precompilation.
- For `ith`, the optional `warmstarts_dir` defaults to `results/heur_warmstart_rand_iter100` in `perform.jl`.

## Outputs

For an instance named `<instance>` and algorithm `<alg>`, the run produces:

- `<out_dir>/<instance>-<alg>/output_trucks.csv`
- `<out_dir>/<instance>-<alg>/output_stacks.csv`
- `<out_dir>/<instance>-<alg>/output_items.csv`
- `<out_dir>/data-<instance>-<alg>.txt`

These files contain:

- `output_trucks.csv`: truck-level summary, including loaded length, loaded weight, loaded volume, and axle-load values
- `output_stacks.csv`: stack placements and extents on the truck floor
- `output_items.csv`: item-level placements inside each stack
- `data-<instance>-<alg>.txt`: solver summary with objective value, bound, relative gap, solve status, and solve time

## Mapping to the paper

The code maps to the paper as follows:

- `IT-0` corresponds to the base integrated item formulation
- `IT` corresponds to the strengthened integrated item formulation with bounds and symmetry breaking
- `IT-H` corresponds to the heuristic warm-started integrated item formulation
- `ST` corresponds to the two-phase stack formulation
- `stack_making.jl` implements the stack-generation heuristic and the exact stack formation model used in the first phase

## Reproducibility notes

- Archived experiment outputs are already included under `results/`.
- `IT-H` reads warm-start CSVs from `results/heur_warmstart_rand_iter100/`; it does not call the Windows heuristic executable during normal use.
- Exact reproducibility may vary with solver version, Gurobi license setup, machine characteristics, and numerical behavior.
- The paper’s experiments were run with a single thread, 16 GB memory limit, and a time limit of 3600 seconds per run.

## License

This repository is distributed under the MIT License. See `LICENSE` for the full text.
