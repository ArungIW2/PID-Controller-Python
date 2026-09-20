# Experiments

Each experiment varies one factor while keeping the mathematical process,
sample time, duration, and setpoint fixed. Run from the repository root after
installing the project:

```bash
python experiments/proportional/run.py
python experiments/integral/run.py
python experiments/derivative/run.py
python experiments/pid/run.py
```

Generated CSV, JSON, and PNG artifacts are written beneath `results/`, which is
ignored by Git unless a curated result is deliberately released.
