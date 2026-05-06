"""Pytest tests for scripts/validate_graph.ps1

Invokes the PowerShell validator via pwsh and asserts on exit codes / output.
"""

import json
import subprocess
import tempfile
from pathlib import Path

import pytest

SCRIPT = Path("/home/geekom/TruckPilot/scripts/validate_graph.ps1")


@pytest.fixture
def tmp_graph():
    """Yield a helper that writes a graph dict to a temp file and returns its path."""
    with tempfile.TemporaryDirectory() as td:
        def _write(graph_dict, name="graph.json"):
            p = Path(td) / name
            p.write_text(json.dumps(graph_dict, indent=2), encoding="utf-8")
            return p

        yield _write


def run_validator(graph_path: Path):
    """Run validate_graph.ps1 and return (completed_process, stdout, stderr)."""
    result = subprocess.run(
        ["pwsh", str(SCRIPT), str(graph_path)],
        capture_output=True,
        text=True,
    )
    return result


class TestHappyPath:
    def test_valid_minimal_graph(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 100.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 0
        assert "7 Prüfungen bestanden" in r.stdout
        assert "0 Verletzungen" in r.stdout

    def test_valid_with_null_road_uid(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": None, "distance_m": 100.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 0
        assert "0 Verletzungen" in r.stdout

    def test_empty_graph(self, tmp_graph):
        path = tmp_graph({"nodes": [], "edges": []})
        r = run_validator(path)
        assert r.returncode == 0


class TestInputErrors:
    def test_missing_file(self, tmp_graph):
        r = subprocess.run(
            ["pwsh", str(SCRIPT), "/nonexistent/path.json"],
            capture_output=True,
            text=True,
        )
        assert r.returncode == 1
        assert "Datei nicht gefunden" in r.stderr

    def test_invalid_json(self, tmp_graph):
        path = tmp_graph({"nodes": [], "edges": []}, name="bad.json")
        path.write_text("{ this is not json", encoding="utf-8")
        r = run_validator(path)
        assert r.returncode == 1
        assert "Ungültiges JSON" in r.stderr

    def test_missing_nodes_key(self, tmp_graph):
        path = tmp_graph({"edges": []})
        r = run_validator(path)
        assert r.returncode == 1
        assert "'nodes' und 'edges' enthalten" in r.stderr

    def test_missing_edges_key(self, tmp_graph):
        path = tmp_graph({"nodes": []})
        r = run_validator(path)
        assert r.returncode == 1
        assert "'nodes' und 'edges' enthalten" in r.stderr


class TestValidationChecks:
    def test_duplicate_node_uid(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x1", "x": 1, "y": 1, "z": 1},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 1.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check0]" in r.stdout
        assert "duplicate node uid" in r.stdout

    def test_missing_node_reference(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x99", "road_uid": "0xA", "distance_m": 100.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check1]" in r.stdout
        assert "fehlt in nodes" in r.stdout

    def test_duplicate_edge(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 100.0},
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 200.0},
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check2]" in r.stdout
        assert "doppelte Edge" in r.stdout

    def test_negative_distance(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": -50}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check3]" in r.stdout
        assert "negative distance_m" in r.stdout

    def test_distance_not_a_number(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": None}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check3]" in r.stdout
        assert "ist keine Zahl" in r.stdout

    def test_non_hex_uid(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "abc", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 100.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check4]" in r.stdout
        assert "ist kein Hex-String" in r.stdout

    def test_self_loop(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x1", "road_uid": "0xA", "distance_m": 100.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check5]" in r.stdout
        assert "Self-loop" in r.stdout

    def test_isolated_node(self, tmp_graph):
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
                {"uid": "0x3", "x": 200, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 100.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "[Check6]" in r.stdout
        assert "isolierter Node" in r.stdout


class TestEdgeCases:
    def test_case_insensitive_uid_normalization(self, tmp_graph):
        """Duplicate UIDs with different casing should be caught."""
        path = tmp_graph({
            "nodes": [
                {"uid": "0xABC", "x": 0, "y": 0, "z": 0},
                {"uid": "0xabc", "x": 1, "y": 1, "z": 1},
            ],
            "edges": [
                {"from_node_uid": "0xABC", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 1.0}
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "duplicate node uid" in r.stdout

    def test_road_uid_with_different_case_same_key(self, tmp_graph):
        """Edges differing only by road_uid casing should be duplicates (case-insensitive)."""
        path = tmp_graph({
            "nodes": [
                {"uid": "0x1", "x": 0, "y": 0, "z": 0},
                {"uid": "0x2", "x": 100, "y": 0, "z": 0},
            ],
            "edges": [
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xA", "distance_m": 100.0},
                {"from_node_uid": "0x1", "to_node_uid": "0x2", "road_uid": "0xa", "distance_m": 200.0},
            ],
        })
        r = run_validator(path)
        assert r.returncode == 2
        assert "doppelte Edge" in r.stdout
