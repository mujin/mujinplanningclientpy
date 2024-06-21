# Changelog

## 0.5.0 (2024-06-19)

- Add `mobilerobotplanningclient`.

## 0.4.0 (2024-06-19)

- Divide the planning client and ZMQ client into separate repositories.

## 0.3.7 (2024-06-02)

- Allow finishStatus/finishMessage to be specified for task StopX commands.

## 0.3.6 (2024-04-30)

- rename to "StartValidatePackFormation" to be more consistent with the threading model.

## 0.3.5 (2024-04-30)

- remove systemState from StartSingleSKUPackFormationComputation

## 0.3.4 (2024-04-01)

### Changes

- Pass controller url, username, and password in userinfo.

## 0.3.3 (2024-03-29)

### Changes

- Add support for async command call for specific functions in `PackingPlanningClient`

## 0.3.2 (2024-02-28)

### Changes

- Add master functions TerminateSlaves, CancelSlaves, Quit

## 0.3.2 (2024-03-11)

### Changes

- Add `ValidatePackFormation` function.

## 0.3.1 (2024-02-28)

### Changes

- Moved GetPackItemPoseInWorld from BinpickingPlanningClient to RealtimeRobotPlanningClient.

## 0.3.0 (2024-02-24)

### Changes

- Add support for async command call.

## 0.1.6 (2023-06-09)

### Changes

- Fix `SetLogLevel` call in planningclient

## 0.1.5 (2023-05-04)

### Changes

- Add `GetPublishedServerState` and `GetPublishedTaskState` functions.

## 0.1.4 (2023-04-06)

### Changes

- Add `ZmqSubscriber`

## 0.1.3 (2023-03-04)

### Changes

- Add `HasDetectionObstacles` function.

## 0.1.2 (2023-02-21)

### Changes

- Add `GetLinkParentInfo` function.

## 0.1.1 (2023-01-30)

### Changes

- Use sensor selections in calibration.

## 0.1.0 (2022-11-17)

### Changes

- Port from mujincontrollerclientpy
