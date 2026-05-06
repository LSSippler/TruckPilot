from __future__ import annotations

import subprocess
import sys


def test_help_flag_prints_usage_and_exits_zero() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "ets2_hashfs", "--help"],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0
    assert "Usage: python -m ets2_hashfs" in result.stdout
