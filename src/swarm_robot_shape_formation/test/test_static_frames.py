"""Fixed-frame refresh derives its geometry from URDF, excluding moving joints."""
import math

from swarm_robot_shape_formation.static_frames import fixed_transforms


def test_only_fixed_joints_are_refreshed_with_exact_prefix_and_origin():
    xml = '''<robot name="test">
      <joint name="rear" type="fixed">
        <parent link="robot_10/base_link"/><child link="robot_10/rear_dock_frame"/>
        <origin xyz="-0.062 0 0" rpy="0 0 3.141592653589793"/>
      </joint>
      <joint name="wheel" type="continuous">
        <parent link="robot_10/base_link"/><child link="robot_10/left_wheel_link"/>
      </joint>
    </robot>'''
    transforms = fixed_transforms(xml)
    assert len(transforms) == 1
    transform = transforms[0]
    assert transform.header.frame_id == 'robot_10/base_link'
    assert transform.child_frame_id == 'robot_10/rear_dock_frame'
    assert transform.transform.translation.x == -.062
    assert abs(transform.transform.rotation.z-1.) < 1e-12
    assert abs(transform.transform.rotation.w) < 1e-12


def test_zero_origin_and_general_roll_pitch_yaw_produce_unit_rotations():
    for origin in ('', '<origin xyz="1 2 3" rpy="0.3 -0.5 0.8"/>'):
        xml = ('<robot><joint type="fixed"><parent link="a"/><child link="b"/>' +
               origin + '</joint></robot>')
        transform = fixed_transforms(xml)[0]
        q = transform.transform.rotation
        assert math.isclose(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w, 1.)
        assert transform.transform.translation.z == (3. if origin else 0.)
