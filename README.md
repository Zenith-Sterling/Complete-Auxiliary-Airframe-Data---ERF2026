# MTR–XV-15 Maneuver Whirl-Flutter Data

This repository contains the auxiliary-airframe aerodynamic data and interpolation
definitions used in the paper:

> **Whirl-Flutter Analysis of a Flexible Tiltrotor Model under Prescribed Steady Maneuvers**

## Fixed paper-linked version

The manuscript should cite the GitHub Release rather than the moving `main` branch:

**https://github.com/YOUR_GITHUB_USERNAME/MTR-XV15-maneuver-flutter-data/releases/tag/v1.0**

Before creating the repository, replace `YOUR_GITHUB_USERNAME` everywhere in this
folder with the actual GitHub username.

## Repository structure

```text
MTR-XV15-maneuver-flutter-data/
├── README.md
├── CITATION.cff
├── LICENSE_NOTICE.md
├── RELEASE_CHECKLIST.md
├── data/
│   ├── table_manifest.csv
│   └── S1_01_...csv through S1_13_...csv
├── docs/
│   ├── complete_auxiliary_airframe_data_and_equations.md
│   └── data_model_overview.md
├── manuscript/
│   └── ERF_Paper_2026_Full_Manuscript_GitHub_Link.md
└── scripts/
    ├── load_tables.py
    └── validate_repository.py
```

## Scope

The data document the XV-15/GTRS auxiliary model used to close the six aircraft-level
trim equations. Baseline rotor and wing loads are supplied by the flexible MTR C81
models. The auxiliary fuselage, tail, control-surface, and drag loads are not applied
in the nonlinear flexible time-domain calculation.

## Recommended release procedure

1. Create a public GitHub repository named `MTR-XV15-maneuver-flutter-data`.
2. Upload the contents of this folder to the repository root.
3. Replace `YOUR_GITHUB_USERNAME` in all files.
4. Review `LICENSE_NOTICE.md` and select the final reuse terms.
5. Commit the files.
6. Create a GitHub Release with tag `v1.0`.
7. Confirm that the Release URL matches the URL used in the manuscript.

## Data format

The `data` folder contains UTF-8 CSV files. The exact equations, units, table titles,
and implementation notes are preserved in
`docs/complete_auxiliary_airframe_data_and_equations.md`.
