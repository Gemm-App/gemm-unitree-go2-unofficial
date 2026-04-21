# Development

## Clone & set up

```bash
git clone https://github.com/Gemm-App/gemm-unitree-go2-unofficial
cd gemm-unitree-go2-unofficial
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
```

The `[dev]` extra pulls in the `[testing]` and `[docs]` extras too, so a
single install gives you the full toolchain.

## Day-to-day commands

| Command                    | What it does                                              |
| -------------------------- | --------------------------------------------------------- |
| `pytest`                   | Runs the offline test suite (50 tests, no hardware)       |
| `ruff check .`             | Lints `src` + `tests`                                     |
| `ruff check . --fix`       | Applies auto-fixable lint suggestions                     |
| `mypy src`                 | Type-checks the package in `strict` mode                  |
| `mkdocs serve`             | Serves the docs at <http://127.0.0.1:8000>                |
| `mkdocs build --strict`    | Builds the docs, failing on warnings                      |
| `python -m build`          | Builds sdist + wheel into `dist/`                         |
| `twine check dist/*`       | Validates distribution metadata before uploading          |

## Testing strategy

- **`tests/conftest.py`** defines a `FakeConnection` that stands in for
  `unitree_webrtc_connect.UnitreeWebRTCConnection`. Tests drive the
  adapter by popping values out of `fake.sent_requests` and pushing fake
  messages back in via `fake.fire(topic, payload)`.
- **`tests/test_contract.py`** runs Gemm's reusable
  `AdapterContractTests` against `UnitreeAdapter`. All 11 contract tests
  must pass.
- **`tests/test_actions.py`** parametrises over the full `SPORT_ACTIONS`
  table to verify every action dispatches to `SPORT_MOD` with the right
  `api_id`.
- **`tests/test_sensors.py`** covers `get_sensor`, `subscribe`, and
  `unsubscribe` for every supported sensor, plus failure modes.
- **`tests/test_adapter.py`** covers constructor validation, lifecycle,
  LiDAR setup, and entry-point discovery.

Nothing talks to real hardware. Any test that would need a robot should
be marked `@pytest.mark.hardware` so it can be skipped with
`pytest -m 'not hardware'` (the default on CI).

## Releasing

1. Bump `__version__` in `src/gemm_unitree_go2_unofficial/__init__.py`
   and `version` in `pyproject.toml`.
2. `rm -rf dist/ && python -m build`.
3. `twine check dist/*`.
4. `twine upload dist/*`.

The docs site is rebuilt from `main` by the GitHub Actions workflow in
`.github/workflows/docs.yml`.
