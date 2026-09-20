# Contributing

## Development setup

```bash
python -m venv .venv
python -m pip install -e ".[dev]"
```

## Quality gate

Before opening a pull request, run:

```bash
python -m ruff check .
python -m mypy src
python -m pytest
```

## Engineering rules

- Keep the PID core independent from plotting, file I/O, and process models.
- Add or update tests in the same change as observable behavior.
- Document the exact discrete equation when changing control calculations.
- Keep experiments deterministic and store their configuration with results.
- Label mathematical-model results honestly; do not imply hardware validation.
- Prefer small phase-sized commits with a single engineering purpose.

