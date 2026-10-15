# Contributing

This is a portfolio record of a completed university project. Bug reports and focused improvements are welcome through GitHub issues or pull requests.

Before submitting code:

```bash
python -m pip install -r requirements-dev.txt
ruff check src tests
pytest -q
```

Hardware-dependent changes should state the Raspberry Pi model, camera, motor driver, GPIO mapping, and test conditions used.
