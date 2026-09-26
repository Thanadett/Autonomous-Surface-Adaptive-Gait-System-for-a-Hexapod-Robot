import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from hexapod_evaluation.scenario import box_model_sdf, parse_model_names, parse_model_position, start_pose, terrain


def test_sdf_is_valid_xml_without_double_quotes() -> None:
    sdf = box_model_sdf("x", (1, 2, 0.1), (0, 0, 0.05, 0, 0, 0), mu=0.3, kp=2000, kd=50)
    assert '"' not in sdf
    root = ET.fromstring(sdf.split("?>", 1)[1])
    assert root.find("model/link/collision/surface/friction/ode/mu").text == "0.3"
    assert root.find("model/link/collision/surface/contact/ode/kp").text == "2000"
    dynamic = ET.fromstring(box_model_sdf("b", (0.1, 0.1, 0.1), (0, 0, 0, 0, 0, 0), static=False).split("?>", 1)[1])
    assert float(dynamic.find("model/link/inertial/mass").text) == 1.0


def test_ramp_top_surface_passes_through_the_start_point() -> None:
    for angle in (0, 10, 25):
        t = terrain({"type": "ramp", "angle_deg": angle}, (-5.0, -1.0), "r")
        root = ET.fromstring(t["sdf"].split("?>", 1)[1])
        px, py, pz, _, pitch, _ = (float(v) for v in root.find("model/pose").text.split())
        a = math.radians(angle)
        n = np.array((-math.sin(a), 0, math.cos(a)))
        top_centre = np.array((px, py, pz)) + n * 0.1        # thickness 0.2 / 2
        g = np.array(t["ground_point"])
        assert abs((g - top_centre) @ n) < 1e-5               # start point lies on the top face (pose printed to 1 um)
        assert pitch == pytest.approx(-a)
        assert t["ground_point"][2] - 0.5 * math.sin(a) == pytest.approx(0.05)  # low end 5 cm up


def test_start_pose_on_a_slope() -> None:
    x, y, z, qx, qy, qz, qw = start_pose((0, 0, 0.1), 20.0, 0.12, 0.0, 0.0, 0.0)
    a = math.radians(20)
    assert (x, z) == pytest.approx((-0.12 * math.sin(a), 0.1 + 0.12 * math.cos(a)))
    assert 2 * math.atan2(qy, qw) == pytest.approx(-a)   # nose up
    assert qx == pytest.approx(0) and qz == pytest.approx(0)


def test_parse_pose_info_text() -> None:
    text = '''header {
  stamp { sec: 3 }
}
pose {
  name: "hexapod"
  id: 9
  position {
    x: -5.4
    y: -1.4
    z: 0.116
  }
}
pose {
  name: "calib_box"
  id: 20
  position {
    x: 3.1
    z: 0.07
  }
  orientation { w: 1 }
}'''
    assert parse_model_position(text, "calib_box") == (3.1, 0.0, 0.07)
    assert parse_model_position(text, "missing") is None


def test_parse_model_names_lists_every_pose_entry():
    text = ('header {\n  stamp {\n    sec: 5\n  }\n}\n'
            'pose {\n  name: "hexapod"\n  id: 8\n  position {\n    x: 1\n  }\n}\n'
            'pose {\n  name: "scn_e4_r10_01"\n  id: 42\n}\n')
    assert parse_model_names(text) == {"hexapod", "scn_e4_r10_01"}
    assert parse_model_names("") == set()
