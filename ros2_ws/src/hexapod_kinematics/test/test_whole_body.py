"""WholeBodyModel: tree FK, centre of mass, support-polygon margin."""
import math

import numpy as np
import pytest

from hexapod_kinematics import WholeBodyModel, convex_hull, stability_margin

URDF = """<robot name="t">
  <link name="world"/>
  <joint name="fix" type="fixed"><parent link="world"/><child link="base_link"/></joint>
  <link name="base_link"><inertial><origin xyz="0 0 0"/><mass value="1.0"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="arm"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <link name="arm"><inertial><origin xyz="0.2 0 0"/><mass value="1.0"/></inertial></link>
</robot>"""


def test_com_follows_the_joint() -> None:
    model = WholeBodyModel.from_urdf(URDF)
    assert model.total_mass == pytest.approx(2.0)
    assert np.allclose(model.com(), (0.15, 0.0, 0.0))            # (0 + 0.3) / 2
    assert np.allclose(model.com({"j1": math.pi / 2}), (0.05, 0.1, 0.0))
    poses = model.link_transforms({"j1": math.pi / 2})
    assert np.allclose(poses["arm"][:3, 3], (0.1, 0.0, 0.0))
    assert "world" not in model.links  # fixed_base's world joint is ignored


def test_hull_and_margin() -> None:
    hull = convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0.5, 0.0)])
    assert len(hull) == 4
    assert stability_margin(hull, (0.5, 0.25)) == pytest.approx(0.25)
    assert stability_margin([(0, 0)], (3, 4)) == pytest.approx(-5.0)
