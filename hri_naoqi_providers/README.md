# hri_naoqi_providers

ROS 2 provider plugins that connect the project's portable capability interfaces to NAOqi-facing services.

## `NaoSpeakRunner`

`hri_naoqi_providers::NaoSpeakRunner` is a Capabilities2 `RunnerBase` plugin. When Capabilities2 starts it, the runner:

1. exposes the configured `hri_capability_interfaces/srv/Speak` resource (or `/hri/speak` when no matching resource is configured);
2. forwards valid requests to `naoqi_utilities_msgs/srv/Say` at `/nao/say`;
3. returns the NAO backend's `success` and `message`, together with the selected provider name.

An empty language selects `Spanish`. The provider always sends `animated=false` and `asynchronous=false`, so requesting speech does not enable animated gestures and the backend reports completion of the utterance.

The adapter is deliberately single-flight: while one speech request is pending, another request receives `provider_busy`. It fails explicitly for blank text and when `/nao/say` is unavailable. The service callback never waits or spins; the portable response is deferred until the asynchronous backend callback runs.

A backend request that does not complete within five seconds returns `nao_say_timeout`. Request identifiers ensure that a late backend response is discarded and cannot complete or corrupt a later request. This initial timeout is intentionally conservative for the short validation phrase and should become configurable before supporting long utterances.

Stopping the runner is idempotent and safe if the backend callback arrives late. A request already accepted by `/nao/say` cannot be cancelled by this provider; stopping only prevents further work and reports `provider_stopped` to an outstanding portable request when the backend completes.

## `NaoqiMoveRelativeRunner`

`hri_naoqi_providers::NaoqiMoveRelativeRunner` exposes
`/hri/move_relative` and delegates bounded ground-plane motion to
`/nao/move_to`. The two internal NAO names can be remapped to equivalent
Pepper interfaces without changing the portable service:

- `/nao/move_to` for the blocking NAOqi motion request;
- `/nao/odom` to verify the achieved displacement.

The provider accepts one request at a time and initially limits targets to
`±0.5 m` in X, `±0.3 m` in Y and `±π/2 rad`. It rejects zero, non-finite and
out-of-range requests before calling the robot. A recent odometry sample is
required before motion. After the backend returns, the provider transforms the
odom delta into the robot's initial frame and checks a `0.05 m` position and
`0.15 rad` angular tolerance.

The provider neither reads nor changes autonomous life. That operational state
remains under the control of the robot launch procedure or the operator.

The underlying `MoveTo` response is empty, so a response from that service is
not considered proof of success. If the 30-second deadline expires, the
portable request fails and the provider enters an uncertain state that rejects
new movement. The local motion lease remains locked until the server process
exits; releasing and reacquiring the capability is not sufficient. Failed
verification and release while motion is pending latch that same lock. Verify
physical stop before restarting. This timeout does not cancel motion: the
current SinfonIA navigation package does not expose `ALMotion.stopMove`.

## `NaoqiMoveVelocityRunner`

Accepts the portable `MoveVelocity` input `/hri/cmd_vel` (`geometry_msgs/Twist`)
and publishes to `/nao/cmd_vel`, remappable to `/pepper/cmd_vel`. Only body-frame
`linear.x`, `linear.y` (m/s) and `angular.z` (rad/s) are supported; limits are
0.10, 0.05 and 0.20 in absolute value. Non-finite, unsupported-axis and excessive
commands cause an immediate zero request, not silent clipping.

Output/status run at 20 Hz. A monotonic 300 ms input watchdog requests zero.
Nonzero commands ramp by at most 0.10 m/s² (X/Y) and 0.40 rad/s² (yaw); stops
bypass ramping. Backend disappearance clears the target; reconnection does not
replay it. QoS is reliable/volatile/keep-last 1. Only one command producer should
be active. There is no obstacle detection or automatic autonomous-life change.

`/hri/move_velocity/status` is a periodic `std_msgs/String`: `ready`, `moving`,
`stopped`, `command_timeout`, `invalid_command`, `backend_unavailable`. These
describe command-side state, **not measured robot motion or physical stop**.

Both motion providers acquire an advisory file lock for the resolved robot
`cmd_vel` namespace when activated. This prevents concurrent cooperating
providers for the same robot on the same host and Unix user. It does not
arbitrate other computers, isolated containers, external teleop or direct
driver calls. Relative motion derives this namespace from its resolved
`move_to` service; use consistent per-robot namespaces/remappings.

Known upstream limitation: Capabilities2 caches the runner before calling
`start()`. If the lease refuses activation, the failed runner can still appear
in `get_running_capabilities`, although its command interfaces were not created.
Check actual interfaces/status and explicitly free the failed entry before retry.
The exclusion test verifies absence of conflicting command interfaces, not merely
the inaccurate running catalog. No upstream code is patched here.

On release the velocity provider publishes zero before destroying its endpoints.
This is best effort: process death, middleware failure or network loss can
prevent delivery. The local driver callback has no independent watchdog. A
robot-side/downstream safety mechanism is required before autonomous operation;
initial physical trials must remain short and directly supervised.

## Dependencies

- ROS 2 Jazzy
- Capabilities2 (`capabilities2_runner`)
- `hri_capability_interfaces`
- SinfonIA's `naoqi_utilities_msgs`
- `nav_msgs`
- `geometry_msgs` and `std_msgs`

The NAOqi bridge must already expose the required robot services and odometry.
This package does not start or configure the robot driver.

## Build

From a ROS 2 workspace containing this package and its dependencies:

```bash
colcon build --packages-select hri_capability_interfaces hri_naoqi_providers
source install/setup.bash
```

This repository's original code is licensed under the MIT License. Dependencies retain their own licenses.
