"""Load all CSV lookup tables in the repository."""

from __future__ import annotations

from pathlib import Path
import csv


def load_tables(data_dir: Path | None = None) -> dict[str, list[dict[str, str]]]:
    """Return all S1 CSV files as dictionaries keyed by filename."""
    if data_dir is None:
        data_dir = Path(__file__).resolve().parents[1] / "data"

    tables: dict[str, list[dict[str, str]]] = {}
    for path in sorted(data_dir.glob("S1_*.csv")):
        with path.open("r", encoding="utf-8-sig", newline="") as file:
            tables[path.name] = list(csv.DictReader(file))
    return tables


if __name__ == "__main__":
    loaded = load_tables()
    for name, rows in loaded.items():
        print(f"{name}: {len(rows)} data rows")
