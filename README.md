# Floating Flywheel Optimal Control (Example)

This repository contains a minimal, reproducible Julia example illustrating
the optimal-control formulation used in the floating flywheel manuscript.

## Files
- `floating_flywheel_example.jl` : Annotated Julia script (JuMP + Ipopt)
- `solution_plot.png`            : Output plot (generated after run)
- `results_traj.csv`             : Output data (t, ω, T, c)
- `README.md`                    : This file

## Requirements
- Julia 1.8+ (1.9 recommended)
- Packages: JuMP, Ipopt, Plots, Statistics, DelimitedFiles

Install packages in Julia:
```julia
using Pkg
Pkg.add(["JuMP","Ipopt","Plots","Statistics","DelimitedFiles"])
```

## Run
```bash
julia floating_flywheel_example.jl
```

## Notes on reproducibility
- Solver tolerance: 1e-8
- Max iterations: 2000
- Grid resolution: N = 1000 (change to test convergence)
- Objective uses trapezoidal integration

For manuscript consistency, the objective sign can be flipped to represent
revenue maximisation.
