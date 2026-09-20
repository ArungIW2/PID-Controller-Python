# Examples

Run examples from the repository root after installing the package:

```bash
python examples/basic_usage.py
python examples/tune.py
```

- `basic_usage.py` runs a PID-controlled first-order mathematical response and
  prints its metrics.
- `tune.py` performs a bounded gain sweep and evaluates the best candidate
  against named process-parameter variations.

These are software-only mathematical studies and do not represent physical
machine tests.

