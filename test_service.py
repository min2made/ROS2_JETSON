#!/usr/bin/env python3
"""
subprocess 방식 서비스 호출 테스트
make sim 실행 후: source install/setup.bash && python3 test_service.py
"""
import os
import subprocess

x, y, yaw = 0.0, 0.0, 0.0
q = (0.0, 0.0, 0.0, 1.0)  # identity quaternion

srv_yaml = (
    f"{{state: {{"
    f"name: 'rccar', "
    f"pose: {{"
    f"position: {{x: {x:.4f}, y: {y:.4f}, z: 0.05}}, "
    f"orientation: {{x: {q[0]:.6f}, y: {q[1]:.6f}, z: {q[2]:.6f}, w: {q[3]:.6f}}}"
    f"}}, "
    f"twist: {{"
    f"linear: {{x: 0.0, y: 0.0, z: 0.0}}, "
    f"angular: {{x: 0.0, y: 0.0, z: 0.0}}"
    f"}}}}}}"
)

print("호출 인자:")
print(srv_yaml)
print()

result = subprocess.run(
    ['ros2', 'service', 'call',
     '/gazebo/set_entity_state',
     'gazebo_msgs/srv/SetEntityState',
     srv_yaml],
    capture_output=True, text=True, timeout=10,
    env=os.environ.copy(),
)

print(f"returncode: {result.returncode}")
print(f"stdout: {result.stdout.strip()}")
if result.stderr:
    print(f"stderr: {result.stderr.strip()}")

if result.returncode == 0:
    print("\n[OK] subprocess 방식 작동! 로봇이 (0,0)으로 이동했어야 함")
else:
    print("\n[FAIL] subprocess 방식도 실패")
