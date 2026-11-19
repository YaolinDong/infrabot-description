# H7C model reference

## Kinematic chain

```text
base_link --Joint_1--> Link_1 --Joint_2--> Link_2 --Joint_3--> Link_3
          --Joint_4--> Link_4 --Joint_5--> Link_5 --Joint_6--> Link_6
          --Joint_7--> Link_7 --Joint_flange (fixed)--> Link_flange
```

- Each joint origin is given in its parent link frame. Every moving joint rotates about its local `z` axis, positive by the right-hand rule.
- RPY angles are fixed-axis roll, pitch, yaw: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
- `Link_flange` is the mounting flange, not a tool TCP. The model has no `world` or `tool0` frame.
- STL vertices are in metres; mesh scale is 1.

## Joints

| Joint | Lower (rad) | Upper (rad) | Damping (N·m·s/rad) | Coulomb friction (N·m) |
|---|---|---|---|---|
| `Joint_1` | -2.8 | 2.8 | 0.1170042814 | 0.6485503358 |
| `Joint_2` | 0 | 3.14 | 0.05456692991 | 0.6212831549 |
| `Joint_3` | -2.8 | 2.8 | 0.3060837998 | 0.4227659217 |
| `Joint_4` | 0 | 3.14 | 0.2433215176 | 0.4968016721 |
| `Joint_5` | -2.8 | 2.8 | 0.01604760697 | 0.09435067175 |
| `Joint_6` | -2 | 2 | 0.01774517461 | 0.1117289144 |
| `Joint_7` | -1.8 | 1.8 | 0.02539361766 | 0.08116653803 |

All joints also declare `effort=100` N·m and `velocity=100` rad/s. These values come from the export and have not been checked against the motors.

`Joint_2` and `Joint_4` have their lower limit at 0, so the zero configuration sits on those limits.

## Zero configuration

With all joints at 0, the flange origin in `base_link` is:

```text
x = 0.052999747919 m
y = 0.000253252621 m
z = 0.220675063983 m
```

The flange orientation is close to identity. `python -B examples/forward_kinematics.py` prints the full transforms.

## Mass and inertia

| Link | Mass (kg) |
|---|---|
| `base_link` | 0.830953007988 |
| `Link_1` | 0.5472459823 |
| `Link_2` | 0.9176600064 |
| `Link_3` | 0.4936541129 |
| `Link_4` | 0.3971543156 |
| `Link_5` | 0.4191733516 |
| `Link_6` | 0.7330713651 |
| `Link_7` | 0.288261214 |
| `Link_flange` | none (frame only) |
| **Total** | **4.627173356** |

Each inertial origin is the link's centre of mass in the link frame, and the inertia tensor is taken about that point. The values for `Link_1`–`Link_7` are identified parameters; `base_link` is fixed and its inertia has no effect on the arm dynamics.

## Friction model

[`config/h7c_friction.yaml`](../config/h7c_friction.yaml) holds the identified joint friction:

```text
tau_f = viscous * dq + coulomb * sign(dq) + offset + spring * q
```

`q` is in rad, `dq` in rad/s, torques in N·m, and `spring` in N·m/rad. `offset` is a constant torque bias, not an angle offset.

The URDF carries only `viscous` (as `damping`) and `coulomb` (as `friction`). URDF loaders ignore `offset` and `spring`; a controller that uses them must read the YAML file.

## Geometry

Visual and collision elements both reference the eight STL files in `meshes/h7c/`. All links use the material `infrabot_gray`.
