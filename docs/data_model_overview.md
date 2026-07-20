# Data and Model Overview

This repository accompanies the paper:

**Whirl-Flutter Analysis of a Flexible Tiltrotor Model under Prescribed Steady Maneuvers**

The repository contains the complete XV-15/GTRS auxiliary-airframe lookup data and
interpolation definitions used to close the aircraft-level algebraic trim equations.
These auxiliary loads are not applied in the nonlinear flexible rotor–wing time-domain
calculation.

## Repository release

Paper-linked release:

https://github.com/YOUR_GITHUB_USERNAME/MTR-XV15-maneuver-flutter-data/releases/tag/v1.0

Replace `YOUR_GITHUB_USERNAME` with the actual GitHub account name before publication.

## Contents

- `data/`: CSV versions of the 13 lookup tables.
- `docs/complete_auxiliary_airframe_data_and_equations.md`: full equations and tables.
- `scripts/load_tables.py`: example loader for all CSV tables.
- `scripts/validate_repository.py`: basic repository validation.
- `manuscript/`: manuscript version containing the direct GitHub Release link.

## Source and scope

The auxiliary model is based on the XV-15 Generic Tilt-Rotor Simulation mathematical
model cited as Ref. 38 in the manuscript. The files here document the subset and
interpolation rules used in the reported calculations.
