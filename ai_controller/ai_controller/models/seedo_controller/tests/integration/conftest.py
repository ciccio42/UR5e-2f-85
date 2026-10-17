from pathlib import Path

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--case-dir",
        action="store",
        default=None,
        help=(
            "Path to a saved real SeeDo test case, "
            "for example /test_real/test_task00_38"
        ),
    )


@pytest.fixture
def case_dir(request) -> Path:
    value = request.config.getoption(
        "--case-dir"
    )

    if value is None:
        pytest.skip(
            "This test requires --case-dir."
        )

    path = (
        Path(value)
        .expanduser()
        .resolve()
    )

    if not path.is_dir():
        raise FileNotFoundError(
            f"Real test case does not exist: {path}"
        )

    return path