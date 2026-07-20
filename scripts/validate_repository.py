"""Perform basic checks on the release repository."""

from __future__ import annotations

from pathlib import Path
import csv
import sys


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
EXPECTED_TABLES = 13


def main() -> int:
    errors: list[str] = []

    csv_files = sorted(DATA.glob("S1_*.csv"))
    if len(csv_files) != EXPECTED_TABLES:
        errors.append(
            f"Expected {EXPECTED_TABLES} lookup CSV files, found {len(csv_files)}."
        )

    for path in csv_files:
        try:
            with path.open("r", encoding="utf-8-sig", newline="") as file:
                rows = list(csv.reader(file))
            if len(rows) < 2:
                errors.append(f"{path.name} has no data rows.")
            if any(len(row) != len(rows[0]) for row in rows[1:]):
                errors.append(f"{path.name} contains rows with inconsistent column counts.")
        except Exception as exc:
            errors.append(f"Could not read {path.name}: {exc}")

    required = [
        ROOT / "README.md",
        ROOT / "CITATION.cff",
        ROOT / "docs" / "complete_auxiliary_airframe_data_and_equations.md",
        ROOT / "data" / "table_manifest.csv",
    ]
    for path in required:
        if not path.exists():
            errors.append(f"Missing required file: {path.relative_to(ROOT)}")

    if errors:
        print("Repository validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print(f"Repository validation passed: {len(csv_files)} lookup tables found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
