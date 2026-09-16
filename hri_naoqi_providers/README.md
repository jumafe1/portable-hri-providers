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

## Dependencies

- ROS 2 Jazzy
- Capabilities2 (`capabilities2_runner`)
- `hri_capability_interfaces`
- SinfonIA's `naoqi_utilities_msgs`

The NAO driver must already expose `/nao/say`. This package does not start or configure the robot driver.

## Build

From a ROS 2 workspace containing this package and its dependencies:

```bash
colcon build --packages-select hri_capability_interfaces hri_naoqi_providers
source install/setup.bash
```

This repository's original code is licensed under the MIT License. Dependencies retain their own licenses.
