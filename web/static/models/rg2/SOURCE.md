# OnRobot RG2 그리퍼 메쉬 (웹 3D 표시용)

- 메쉬: `onrobot-ros2/onrobot_rg_description/meshes/rg2/visual/*.stl` (MIT, LICENSE 참고). 단위 m → robot3d.js 에서 ×1000
- 붙는 위치·관절: `m0609_rg2_bringup/urdf/m0609_with_rg2.urdf.xacro`, `onrobot_rg2.xacro`, `onrobot_rg2_model_macro.xacro`
  - `link_6 → tool0` rpy (π, −π/2, 0), `tool0 → rg2_base_link` rpy (π/2, 0, π/2)
  - 손가락: finger_joint 와 mimic 관절 (outer/inner knuckle, inner finger) → robot3d.js `buildRG2()`
