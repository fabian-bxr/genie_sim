# Genie G2 development setup — working notes

Operational notes for running the Genie Sim RT Engine (`geniesim_ros`) against the
AgiBot Genie G2 on this machine, plus the asset problems hit on first bring-up and
how they were resolved.

Written 2026-10-06. Verified against commit `6ca11c7`, `geniesim 3.2.0`,
Isaac Sim container `registry.agibot.com/genie-sim/geniesim3:latest`.

---

## 1. Host environment

| | |
|---|---|
| GPU | NVIDIA RTX 5080 (16 GB), driver 615.71.09 |
| Docker | 29.8.2 |
| Host venv | `.venv` (uv, Python 3.13) — holds `geniesim_cli` + `geniesim_assets` editable |
| Assets | `assets/` (~28 GB), already present |
| Container | `geniesim3` — Isaac Sim + ROS 2 Jazzy |

The host venv deliberately does **not** have `geniesim_ros` / `geniesim_benchmark`;
those live inside the container. `geniesim version` reporting them as "not installed"
on the host is expected, not a problem.

### One-time host setup

The `geniesim` console script is not on `PATH` after a uv install. Symlink it:

```bash
ln -s /home/fabian/PycharmProjects/genie_sim/.venv/bin/geniesim ~/.local/bin/geniesim
```

The shim has an absolute shebang into the venv Python and resolves the repo root by
walking up from `__file__`, so it works from any cwd — no `GENIESIM_REPO_ROOT` export
needed.

**Fetch Git LFS before anything else.** Meshes (`*.STL`, `*.dae`, `*.obj`, `*.ply`,
`*.glb`, `*.gltf`, `*.fbx`) are LFS-tracked. See § 4.2 for what happens if you skip this.

```bash
sudo pacman -S git-lfs && git lfs install && git lfs pull
```

---

## 2. Running the sim

Everything below runs **inside** the container.

```bash
geniesim docker up && geniesim docker into
```

`geniesim docker` aliases to `docker5.1` → the `geniesim3` container. `docker6.0`
exists but refuses to dispatch — the Dockerfile is a placeholder and the image is
not published.

Inside the container:

```bash
cd /workspace && source /opt/ros/jazzy/setup.bash && source devel/setup.bash
```

The ROS workspace is already built (`devel/`, `devel_build/`, `devel_log/`). Rebuild
only after editing ROS sources:

```bash
geniesim ros build dev
```

### Scene + MoveIt

Two terminals, or background the first.

```bash
ros2 launch genie_sim_bringup app.launch.py scene:=scene_pnp_g2_op launcher_config:=launcher_ovrtx_isaac_physx headless:=false
```

```bash
ros2 launch genie_sim_moveit wbc.launch.py arm:=crs gripper:=omnipicker
```

The `arm:` / `gripper:` args must match the scene's robot variant — `scene_pnp_g2_op`
stages `G2_crs_omnipicker`, so `arm:=crs gripper:=omnipicker`. Mismatching them gives
MoveIt a different URDF than the simulator is running.

| Scene | Robot | Shows |
|---|---|---|
| `scene_pnp_g2_op` | G2 + crs arm + omnipicker | pick-and-place |
| `scene_wbc_g2_sp` | G2 + swiftpicker | whole-body control |
| `scene_flat_g2_sp` | G2 + swiftpicker | flat/empty world |

| Launcher | Physics | Renderer |
|---|---|---|
| `launcher_ovrtx_isaac_physx` | Isaac PhysX | standalone OVRtx node |
| `launcher_newton_mjwarp` | Newton-standalone | inline OVRtx |

Isaac Sim takes a few minutes to come up on first launch after a container restart
(shader cache warmup). The engine prints per-500-step physics stats once it is
stepping.

### Startup is healthy when

```bash
ros2 topic echo /joint_states --once    # 46 joints, non-empty name/position
ros2 topic echo /tf --once              # non-empty transforms
```

Known-good numbers for `scene_pnp_g2_op`:

| Signal | Value |
|---|---|
| `/joint_states` | 46 joints |
| `/tf` | 49 unique moving frames |
| `/tf_static` | 9 frames |
| `/clock` | ~90–100 Hz |
| `/moveit/joint_states` | ~79 Hz |
| cameras | 20–24 Hz |
| ros2_control controllers | 10, all `active` |

Engine log lines that indicate a correctly loaded robot:

```
[fix_base] enabled world-weld /genie/Physics/root_joint (body0=/genie, body1=/genie/Geometry/base_link)
[classify] body=5 arm=0 head=3 gripper=16 chassis_drive=4 chassis_steer=4
[stage] active physics engine: PhysX
```

`arm=0` is benign — it is drive-tuning bookkeeping, and the arm joints take the
default drive path. They are driven, not limp: with the stack running they hold the
scene's `init_joint_pos` to within 0.01°.

`ros2 control` is **not** installed in this container. To inspect controllers:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

---

## 3. The staged-asset pipeline (needed to read § 4)

On launch, `app.launch.py` runs a two-stage bake into
`assets/scenes/<scene_stem>/` — for this scene, `assets/scenes/scene_pnp_g2_op/`:

1. **`assemble_robot.py`** — xacro → URDF → USD. Writes `robot.urdf`, `robot.usda`,
   and a `payloads/` tree. Runs a separate headless Kit app; takes a few minutes.
2. **`assemble_scene.py`** — composes robot + scene + render layer, writes
   `manifest.json`.

Both are **cached**. `assemble_robot` is skipped when its artefacts exist and the
URDF sha256 in `urdf.sha256` matches; `manifest.json` is never cached. Force a full
rebuild with `always_regenerate_robot_usd:=true`.

Because the bake is cached, **a bad bake is sticky** — it survives container
restarts, workspace rebuilds, and relaunches. Both problems in § 4 were sticky in
exactly this way.

Which `robot.usda` gets loaded is decided at
[`assemble_scene.py:485`](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/assemble_scene.py):
if the scene yaml's `robot.robot_source` has a `urdf` key (even empty, as
`scene_pnp_g2_op.yaml` does), the **staged** USD is used. Otherwise the pre-baked
`assets/robot/<robot_name>/robot.usda` is used. There is no silent fallback between
the two.

---

## 4. Problems hit on first bring-up

Both produced the same headline symptom — **no robot** — but they are independent
and have different fixes. They stack: fixing one reveals the other.

### 4.1 No `Physics` variant selection → robot has no bodies or joints

**Symptom.** Isaac Sim viewport empty, RViz empty. The sim was otherwise healthy:
`/clock` at ~90 Hz, 41,000 physics steps, 12,000 rendered frames, GPU at 91%.

**Discriminating evidence.** `/joint_states` published *at rate* with **empty**
`name` and `position` arrays, and `/tf` published empty `transforms`. A robot that
fails to load is silent; a robot that loads with zero joints publishes empty
messages. The engine log said it directly:

```
WARN: [fix_base] no RigidBodyAPI descendant under /genie; cannot identify URDF root link
[classify] body=0 arm=0 head=0 gripper=0 chassis_drive=0 chassis_steer=0 other=0
```

**Cause.** The staged `robot.usda` wraps all physics in a `Physics` variantSet
(`mujoco | none | physics | physx`) and authors **no selection**. With no selection,
nothing composes:

| `Physics` selection | rigid bodies | joints |
|---|---|---|
| *(none — as generated)* | 0 | 0 |
| `physx` | 56 | 46 |
| `physics` | 56 | 46 |
| `none` | 0 | 0 |

Nothing in the ROS workspace selects a variant — `SetVariantSelection` appears
nowhere under `src/`. The joint-collection fallback at
[`stage.py:604`](../source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/kit/stage.py)
assumes AS3 joints "live in `payloads/Physics/physx.usda` (a sublayer)" and will
compose once Kit processes sublayers — but they are behind a *variant + payload*,
so they never compose and the fallback finds nothing either.

**This is a standing bug, not a one-off.** A clean regeneration with correct meshes
produced a `robot.usda` with no selection again.

**Fix.** Author the selection in `assets/scenes/scene_pnp_g2_op/robot.usda`, inside
the `def Xform "robot" (...)` metadata block:

```
def Xform "robot" (
    prepend references = @./payloads/base.usda@
    append variantSets = "Physics"
    variants = {
        string Physics = "physx"
    }
)
```

Or from the host venv (which has `pxr`):

```bash
.venv/bin/python -c "from pxr import Usd; s=Usd.Stage.Open('assets/scenes/scene_pnp_g2_op/robot.usda'); s.GetDefaultPrim().GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx'); s.GetRootLayer().Save()"
```

Match the selection to the launcher: `physx` for `launcher_ovrtx_isaac_physx`,
`mujoco` for `launcher_newton_mjwarp`. Both `physx.usda` and `mujoco.usda` sublayer
the shared `physics.usda`.

**Recurrence.** The fix lives in a generated file. It is wiped by
`always_regenerate_robot_usd:=true`, by deleting `assets/scenes/<scene>/`, and by
anything that changes the URDF sha256. Re-apply after any rebake.

### 4.2 Git LFS stubs at bake time → robot has no visual geometry

**Symptom.** After 4.1 was fixed: robot correct in RViz, still invisible in Isaac Sim.

**Cause.** The URDF's 51 visual elements reference `.dae` meshes, and `*.dae` is
LFS-tracked. The first bake ran before `git-lfs` was installed, so every mesh was a
~130-byte pointer stub. The converter had no geometry to import. Physics survived
because joints and inertias come from the URDF text, not from meshes.

The resulting USD contained **zero** `Mesh` prims and zero vertex data — only 38
`Sphere` prims named `*_coarse_N`, carrying `PhysicsCollisionAPI` and marked
`uniform token purpose = "guide"`. Guide-purpose prims are not rendered. The robot
was physically present, fully articulated, holding its commanded pose — and made
entirely of invisible collision spheres.

RViz was unaffected throughout because it never touches the USD: it builds its model
from the URDF and loads meshes straight off disk. **RViz looking correct does not
mean the simulated asset is correct** — the two render paths share no data.

**Fix.** Fetch LFS, then force a rebake, then re-apply § 4.1, then relaunch:

```bash
git lfs install && git lfs pull
```

```bash
ros2 launch genie_sim_bringup app.launch.py scene:=scene_pnp_g2_op launcher_config:=launcher_ovrtx_isaac_physx headless:=false always_regenerate_robot_usd:=true
```

That launch will come up with the § 4.1 empty-robot symptom, because regeneration
overwrites `robot.usda` and drops the variant selection. Re-apply the selection, then
relaunch **without** `always_regenerate_robot_usd`.

**Before / after**, same scene, composed stage:

| | bad bake | after refetch + rebake |
|---|---|---|
| `payloads/geometries.usd` | *absent* | 9.3 MB |
| `payloads/instances.usda` | *absent* | 48 KB |
| `payloads/materials.usda` | *absent* | 15 KB |
| meshes | 0 | 64 (in 74 instance prototypes) |
| vertices | 0 | 444,821 |
| rigid bodies | 56 | 56 |
| joints | 46 | 46 |

Note that `Usd.Stage.Traverse()` reports `meshes=0` even on a good asset — AS3 uses
instanceable prims and `Traverse()` does not descend into instance proxies. Count
via `stage.GetPrototypes()` instead.

**Also note:** the container writes regenerated assets as root. Restore ownership
afterwards or host-side tools will fail with permission errors:

```bash
docker exec geniesim3 chown -R 1000:1000 /workspace/assets/scenes/scene_pnp_g2_op
```

### 4.3 LFS stubs at runtime → MoveIt has no collision meshes

Separate consequence of the same missing LFS fetch, visible in the MoveIt log as
408 × `Unable to read file, malformed XML` plus repeated `Failed to determine STL
storage representation`. All 168 `.STL` files were pointer stubs.

This does not affect the simulation — Isaac drives physics from the USD. It does
mean MoveIt plans **without** the gripper and head convex hulls, so planned
trajectories cannot be trusted until it is fixed. Cleared by `git lfs pull`
(0 mesh errors afterwards).

---

## 5. Benign log noise

Not worth chasing:

| Message | Why it's fine |
|---|---|
| `The link head_link1 / body_link4 has unrealistic inertia` | RViz declining to draw the inertia box. [`recompute_g2_inertia.py`](../source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model/scripts/recompute_g2_inertia.py) exists if it ever matters. |
| `Group 'chassis' does not have a parent link` | Expected — the chassis is an SRDF planar virtual joint driven by `PlanarBaseController`, not a kinematic chain. |
| `No 3D sensor plugin(s) defined for octomap updates` | Optional MoveIt perception, not configured in this scene. |
| `Action server: /recognize_objects not available` | As above. |
| `Viewport(in-loop): ... 0 is normal for ovrtx` | Deliberate — the Isaac viewport is deprioritized when the standalone OVRtx node renders. |
| `[classify] arm=0` | Drive-tuning bookkeeping. Arms are driven; they hold `init_joint_pos` to 0.01°. |

---

## 6. Open items

Neither is understood; both were seen with an otherwise-healthy stack after § 4 was
fixed and the robot confirmed visible in Isaac Sim.

- **`/genie_sim/head_front_camera_rgb` renders fully black.** Plausibly correct —
  that camera sits inside `head_link3`, which now has a mesh around it where before
  there was nothing to occlude it — but unverified. No before-image exists to
  compare against. The free camera renders the scene correctly (textures, lighting,
  shadows), so the renderer itself works.
- **Publishing `geometry_msgs/PoseStamped` to
  `/genie_sim_engine/viewer/camera_pose` had no visible effect** on the free camera.
  The topic exists with one subscriber. The intended driver is the RViz2 free-camera
  pose plugin; driving it by raw topic publish may need a different frame convention
  or may not be supported.

---

## 7. Interfacing with the robot

`geniesim_ros` presents the simulator as an ordinary ROS 2 node — `/joint_states`,
`/joint_command` (both `sensor_msgs/JointState`), `/tf`, `/clock`, `/odom`,
`/cmd_twist`, `/robot_description`, per-camera `image_raw` + `camera_info`,
`/set_servo_mode` — plus a real `ros2_control` `SystemInterface` plugin, so MoveIt
treats it like any other hardware backend. Control logic written against the sim
ports to a ROS-fronted G2 unchanged.

Caveat: AgiBot's production G2 stack is its own middleware, vendored here at
[`source/data_collection/common/aimdk/protocol`](../source/data_collection/common/aimdk/protocol)
with `hal/` and `sim/` namespaces. The topic surface above is Genie Sim's contract,
not the robot's native transport. Deploying onto the native stack means an adapter
at the edge.

Two other stacks in this repo are **not** this interface — don't conflate them:

- `geniesim_benchmark` — drives Isaac Sim directly over gRPC with a WebSocket
  inference-server protocol, for policy evaluation.
- `data_collection` — cuRobo trajectory planning + gRPC, records agibot-format
  episodes.
