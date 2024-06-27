
# This file contains the types used in the client
from typing import TypedDict, Union, Any, Optional, Literal
from typing_extensions import Required

PingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PingParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PingParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PingParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PingParametersDynamicEnvironmentStateMapValue = TypedDict('PingParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PingParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PingParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PingParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

PingReturns = TypedDict('PingReturns', {
    'timestamp': float,
    'slaverequestid': str,
}, total=False)

GetJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetJointValuesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetJointValuesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetJointValuesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetJointValuesParametersDynamicEnvironmentStateMapValue = TypedDict('GetJointValuesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetJointValuesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetJointValuesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetJointValuesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetJointValuesParametersRobotBridgeConnectionInfo = TypedDict('GetJointValuesParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetJointValuesParametersLocationCollisionInfosArrayElement = TypedDict('GetJointValuesParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

GetJointValuesReturnsToolsMapValue = TypedDict('GetJointValuesReturnsToolsMapValue', {
    'translate': tuple[float, float, float],
    'quaternion': list[float],
    'indices': list[int],
}, total=False)

GetJointValuesReturnsTools = dict[str, dict[str, Any]]

GetJointValuesReturnsLinksMapValue = TypedDict('GetJointValuesReturnsLinksMapValue', {
    'translate': tuple[float, float, float],
    'quaternion': list[float],
}, total=False)

GetJointValuesReturnsLinks = dict[str, dict[str, Any]]

GetJointValuesReturns = TypedDict('GetJointValuesReturns', {
    'currentjointvalues': list[float],
    'jointNames': list[str],
    'tools': GetJointValuesReturnsTools,
    'links': GetJointValuesReturnsLinks,
}, total=False)

MoveToolLinearParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveToolLinearParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveToolLinearParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveToolLinearParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveToolLinearParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveToolLinearParametersDynamicEnvironmentStateMapValue = TypedDict('MoveToolLinearParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveToolLinearParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveToolLinearParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveToolLinearParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveToolLinearParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveToolLinearParametersRobotBridgeConnectionInfo = TypedDict('MoveToolLinearParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveToolLinearParametersLocationCollisionInfosArrayElement = TypedDict('MoveToolLinearParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

MoveStraightParameters = TypedDict('MoveStraightParameters', {
    'closeConfigurationThresh': float,
    'closeIkThresh': float,
    'dumpFailedTrajectoryLevel': int,
    'cosDeltaAngleThresh': float,
    'dynamicsConstraintsType': Literal['IgnoreTorque', 'NominalTorque', 'InstantaneousTorque'],
    'ikMidpointMaxDistMult': float,
    'ikMidpointMaxTransDist': float,
    'ikMidpointMaxRotDist': float,
    'jacobianMinEigenValue': float,
    'jacobianMinCond': float,
    'maxAdaptiveSteps': int,
    'maxRevoluteDifference': float,
    'moveStraightMode': str,
    'numMaxTries': int,
    'trajSampleTimeStep': float,
    'rotationDiffThresh': float,
    'translationDiffThresh': float,
    'useworkspaceplanner': int,
    'worksteplength': float,
    'jointaccelmult': float,
    'maxIterations': int,
    'workVerifyMaxAdaptiveSteps': int,
    'slowDownMult': float,
    'startMoveLinearOffsetDir': tuple[float, float, float],
    'startMoveLinearOffsetInTool': bool,
    'workmaxdeviationangle': float,
    'workignorefirstcollision': float,
    'workignorefirstcollisionee': float,
    'workignorelastcollisionee': float,
    'workVerifyStepLength': float,
}, total=False)

MoveToHandPositionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveToHandPositionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveToHandPositionParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveToHandPositionParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveToHandPositionParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveToHandPositionParametersDynamicEnvironmentStateMapValue = TypedDict('MoveToHandPositionParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveToHandPositionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveToHandPositionParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveToHandPositionParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveToHandPositionParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveToHandPositionParametersRobotBridgeConnectionInfo = TypedDict('MoveToHandPositionParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveToHandPositionParametersLocationCollisionInfosArrayElement = TypedDict('MoveToHandPositionParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

PathPlannerParameters = TypedDict('PathPlannerParameters', {
    'dynamicsConstraintsType': Literal['IgnoreTorque', 'NominalTorque', 'InstantaneousTorque'],
    'maxiter': int,
    'maxPlanningTime': float,
    'pathPlannerName': Literal['BiRRT', 'DualSpaceTreeSearch'],
    'steplength': float,
    'goalBiasProb': float,
    'workspaceSamplingBiasProb': float,
    'workspaceStepLength': float,
}, total=False)

ForceTorqueFilterParameters = TypedDict('ForceTorqueFilterParameters', {
    'type': Literal['butterworth', 'bessel'],
    'cutOffFrequency': float,
    'order': int,
}, total=False)

SensorBiasPerIOValue = TypedDict('SensorBiasPerIOValue', {
    'id': str,
    'deleted': bool,
    'index': int,
    'iovalue': int,
    'forceTorque': list[float],
}, total=False)

SensorBiasByIoSignals = TypedDict('SensorBiasByIoSignals', {
    'id': str,
    'deleted': bool,
    'index': int,
    'ioname': str,
    'sensorBiasPerIoValue': list[SensorBiasPerIOValue],
    'type': Literal['onlyMatch', 'linearInterpolation'],
}, total=False)

ForceTorqueBasedEstimatorParameters = TypedDict('ForceTorqueBasedEstimatorParameters', {
    'allowRobotMovementTowardDecreasingExternalForce': bool,
    'collisionExternalForceTorqueThresholds': list[float],
    'collisionSensorValueThresholds': list[float],
    'controlDelay': float,
    'correctionMatrix': tuple[tuple[float, float, float, float, float, float], tuple[float, float, float, float, float, float], tuple[float, float, float, float, float, float], tuple[float, float, float, float, float, float], tuple[float, float, float, float, float, float], tuple[float, float, float, float, float, float]],
    'deviceId': str,
    'devicename': str,
    'externalForceTorqueFilter': ForceTorqueFilterParameters,
    'forceTorqueFilter': ForceTorqueFilterParameters,
    'handCenterOfMass': list[float],
    'handMass': float,
    'measurementTimeout': float,
    'maxAllowedNumConsecutiveCollisionDetection': int,
    'minVerticalAccelerationForMassEstimation': float,
    'minVerticalForceForCenterOfMassEstimation': float,
    'objectCenterOfMass': list[float],
    'objectMass': float,
    'objectMassEstimationFilter': ForceTorqueFilterParameters,
    'robotForceTorqueCalibrationPositionName': str,
    'robotForceTorqueCalibrationPositionNames': list[str],
    'sensorBiasByIoSignals': list[SensorBiasByIoSignals],
    'sensorForceAccuracy': float,
    'sensorTorqueAccuracy': float,
    'sensorFrequency': float,
    'sensorLinkName': str,
    'sensorPolarity': Literal[-1, 1],
    'use': bool,
    'useCorrectionMatrix': bool,
    'useCommandedJointValues': bool,
}, total=False)

JittererParameters = TypedDict('JittererParameters', {
    'maxJitter': float,
    'maxJitterIterations': int,
    'maxJitterLinkDist': float,
    'jitterBiasDirection': tuple[float, float, float],
    'jitterNeighDistThresh': float,
    'useWorkspaceJitterer': bool,
}, total=False)

MoveToHandPositionParametersGripperInfoVariantItemPrefix0 = dict[str, Any]

MoveToHandPositionParametersStartJointConfigurationStatesArrayElement = TypedDict('MoveToHandPositionParametersStartJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveToHandPositionParametersGoalJointConfigurationStatesArrayElement = TypedDict('MoveToHandPositionParametersGoalJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SmoothingParameters = TypedDict('SmoothingParameters', {
    'linearSmoothingIterations': int,
    'minSmootherIterations': int,
    'smootheriterations': int,
    'smootherplannername': str,
    'smoothingStepLength': float,
    'smoothingDurationImprovementCutoffRatio': float,
    'smoothingPointTolerance': float,
    'smoothingTimeLimit': float,
}, total=False)

GrabParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GrabParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GrabParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GrabParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GrabParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GrabParametersDynamicEnvironmentStateMapValue = TypedDict('GrabParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GrabParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GrabParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GrabParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GrabParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ReleaseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ReleaseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ReleaseParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ReleaseParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ReleaseParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ReleaseParametersDynamicEnvironmentStateMapValue = TypedDict('ReleaseParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ReleaseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ReleaseParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ReleaseParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ReleaseParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetGrabbedParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetGrabbedParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetGrabbedParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetGrabbedParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetGrabbedParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetGrabbedParametersDynamicEnvironmentStateMapValue = TypedDict('GetGrabbedParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetGrabbedParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetGrabbedParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetGrabbedParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetGrabbedParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetGrabbedReturns = TypedDict('GetGrabbedReturns', {
    'names': Optional[list[str]],
}, total=False)

GetTransformParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetTransformParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetTransformParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetTransformParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetTransformParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetTransformParametersDynamicEnvironmentStateMapValue = TypedDict('GetTransformParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetTransformParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetTransformParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetTransformParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetTransformParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetTransformReturns = TypedDict('GetTransformReturns', {
    'translation': list[float],
    'rotationmat': list[list[float]],
    'quaternion': list[float],
}, total=False)

GetLinkParentInfoParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetLinkParentInfoParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetLinkParentInfoParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetLinkParentInfoParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetLinkParentInfoParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetLinkParentInfoParametersDynamicEnvironmentStateMapValue = TypedDict('GetLinkParentInfoParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetLinkParentInfoParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetLinkParentInfoParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetLinkParentInfoParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetLinkParentInfoParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetLinkParentInfoReturns = TypedDict('GetLinkParentInfoReturns', {
    'name': str,
    'translation': list[Any],
    'rotationmat': list[Any],
    'quaternion': list[Any],
}, total=False)

SetTransformParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetTransformParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetTransformParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetTransformParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetTransformParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetTransformParametersDynamicEnvironmentStateMapValue = TypedDict('SetTransformParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetTransformParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetTransformParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetTransformParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetTransformParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetOBBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetOBBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetOBBParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetOBBParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetOBBParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetOBBParametersDynamicEnvironmentStateMapValue = TypedDict('GetOBBParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetOBBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetOBBParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetOBBParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetOBBParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetOBBReturns = TypedDict('GetOBBReturns', {
    'extents': Any,
    'boxLocalTranslation': Any,
    'originalBodyTranslation': Any,
    'quaternion': Any,
    'rotationmat': Any,
    'translation': Any,
}, total=False)

GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValue = TypedDict('GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetInnerEmptyRegionOBBParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetInnerEmptyRegionOBBParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetInnerEmptyRegionOBBReturns = TypedDict('GetInnerEmptyRegionOBBReturns', {
    'extents': Any,
    'boxLocalTranslation': Any,
    'originalBodyTranslation': Any,
    'quaternion': Any,
    'rotationmat': Any,
    'translation': Any,
}, total=False)

GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValue = TypedDict('GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetInstObjectAndSensorInfoParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetInstObjectAndSensorInfoParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValue = TypedDict('GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetInstObjectInfoFromURIParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetInstObjectInfoFromURIParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetInstObjectInfoFromURIReturnsObb = TypedDict('GetInstObjectInfoFromURIReturnsObb', {
    'transform': tuple[float, float, float, float, float, float, float],
    'extents': tuple[float, float, float],
}, total=False)

GetInstObjectInfoFromURIReturnsInnerobb = TypedDict('GetInstObjectInfoFromURIReturnsInnerobb', {
    'transform': tuple[float, float, float, float, float, float, float],
    'extents': tuple[float, float, float],
}, total=False)

Geometry = TypedDict('Geometry', {
    'id': str,
    'name': str,
    'type': Literal['mesh', 'box', 'container', 'cage', 'sphere', 'cylinder', 'axial', 'trimesh', 'calibrationboard', 'conicalfrustum', ''],
    'diffuseColor': list[float],
    'outerExtents': list[float],
    'innerExtents': list[float],
    'halfExtents': list[float],
    'transparency': float,
    'negativeCropContainerMargins': list[float],
    'negativeCropContainerEmptyMargins': list[float],
    'positiveCropContainerMargins': list[float],
    'positiveCropContainerEmptyMargins': list[float],
    'transform': tuple[float, float, float, float, float, float, float],
}, total=False)

IkParameterizationCustomDataArrayElement = TypedDict('IkParameterizationCustomDataArrayElement', {
    'id': str,
    'values': list[float],
}, total=False)

IkParameterization = TypedDict('IkParameterization', {
    'id': str,
    'name': str,
    'type': Literal['None', 'Transform6D', 'Rotation3D', 'Translation3D', 'Direction3D', 'Ray4D', 'Lookat3D', 'TranslationDirection5D', 'TranslationXY2D', 'TranslationXYOrientation3D', 'TranslationLocalGlobal6D', 'TranslationXAxisAngle4D', 'TranslationYAxisAngle4D', 'TranslationZAxisAngle4D', 'TranslationXAxisAngleZNorm4D', 'TranslationYAxisAngleXNorm4D', 'TranslationZAxisAngleYNorm4D'],
    'transform': tuple[float, float, float, float, float, float, float],
    'quaternion': tuple[float, float, float, float],
    'translate': tuple[float, float, float],
    'localTranslate': tuple[float, float, float],
    'direction': tuple[float, float, float],
    'angle': float,
    'customData': list[IkParameterizationCustomDataArrayElement],
}, total=False)

GetInstObjectInfoFromURIReturns = TypedDict('GetInstObjectInfoFromURIReturns', {
    'translation': tuple[float, float, float],
    'rotationmat': tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    'quaternion': tuple[float, float, float, float],
    'obb': GetInstObjectInfoFromURIReturnsObb,
    'innerobb': GetInstObjectInfoFromURIReturnsInnerobb,
    'geometryInfos': list[Geometry],
    'ikparams': list[IkParameterization],
}, total=False)

GetAABBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetAABBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetAABBParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetAABBParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetAABBParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetAABBParametersDynamicEnvironmentStateMapValue = TypedDict('GetAABBParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetAABBParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetAABBParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetAABBParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetAABBParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

AABB = TypedDict('AABB', {
    'pos': tuple[float, float, float],
    'extents': tuple[float, float, float],
}, total=False)

SetLocationTrackingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetLocationTrackingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetLocationTrackingParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetLocationTrackingParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetLocationTrackingParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetLocationTrackingParametersDynamicEnvironmentStateMapValue = TypedDict('SetLocationTrackingParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetLocationTrackingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetLocationTrackingParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetLocationTrackingParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetLocationTrackingParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetLocationTrackingParametersIoSignalsInfoVariantItemPrefix0 = dict[str, Any]

ResetLocationTrackingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResetLocationTrackingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResetLocationTrackingParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResetLocationTrackingParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResetLocationTrackingParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResetLocationTrackingParametersDynamicEnvironmentStateMapValue = TypedDict('ResetLocationTrackingParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResetLocationTrackingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResetLocationTrackingParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResetLocationTrackingParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResetLocationTrackingParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValue = TypedDict('GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetLocationTrackingInfosParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetLocationTrackingInfosParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetLocationTrackingInfosReturns = dict[str, Any]

UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValue = TypedDict('UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': UpdateLocationContainerIdTypeParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

UpdateLocationContainerIdTypeParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValue = TypedDict('ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResetLocationTrackingContainerIdParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResetLocationTrackingContainerIdParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValue = TypedDict('RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RemoveObjectsWithPrefixParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RemoveObjectsWithPrefixParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RemoveObjectsWithPrefixParametersRobotBridgeConnectionInfo = TypedDict('RemoveObjectsWithPrefixParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

RemoveObjectsWithPrefixReturns = TypedDict('RemoveObjectsWithPrefixReturns', {
    'removedBodyNames': Any,
}, total=False)

GetTrajectoryLogParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetTrajectoryLogParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetTrajectoryLogParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetTrajectoryLogParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetTrajectoryLogParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetTrajectoryLogParametersDynamicEnvironmentStateMapValue = TypedDict('GetTrajectoryLogParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetTrajectoryLogParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetTrajectoryLogParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetTrajectoryLogParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetTrajectoryLogParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetTrajectoryLogReturnsTrajectoriesArrayElement = TypedDict('GetTrajectoryLogReturnsTrajectoriesArrayElement', {
    'timestarted': int,
    'name': str,
    'numpoints': int,
    'duration': float,
    'timedjointvalues': list[Any],
}, total=False)

GetTrajectoryLogReturns = TypedDict('GetTrajectoryLogReturns', {
    'total': int,
    'trajectories': list[GetTrajectoryLogReturnsTrajectoriesArrayElement],
}, total=False)

ChuckGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ChuckGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ChuckGripperParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ChuckGripperParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ChuckGripperParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ChuckGripperParametersDynamicEnvironmentStateMapValue = TypedDict('ChuckGripperParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ChuckGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ChuckGripperParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ChuckGripperParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ChuckGripperParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

UnchuckGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('UnchuckGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

UnchuckGripperParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

UnchuckGripperParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

UnchuckGripperParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

UnchuckGripperParametersDynamicEnvironmentStateMapValue = TypedDict('UnchuckGripperParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[UnchuckGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': UnchuckGripperParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': UnchuckGripperParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

UnchuckGripperParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

UnchuckGripperParametersRobotBridgeConnectionInfo = TypedDict('UnchuckGripperParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

UnchuckGripperParametersLocationCollisionInfosArrayElement = TypedDict('UnchuckGripperParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

CalibrateGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('CalibrateGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

CalibrateGripperParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

CalibrateGripperParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

CalibrateGripperParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

CalibrateGripperParametersDynamicEnvironmentStateMapValue = TypedDict('CalibrateGripperParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[CalibrateGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': CalibrateGripperParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': CalibrateGripperParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

CalibrateGripperParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StopGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopGripperParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopGripperParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopGripperParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopGripperParametersDynamicEnvironmentStateMapValue = TypedDict('StopGripperParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopGripperParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopGripperParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopGripperParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveGripperParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveGripperParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveGripperParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveGripperParametersDynamicEnvironmentStateMapValue = TypedDict('MoveGripperParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveGripperParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveGripperParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveGripperParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveGripperParametersRobotBridgeConnectionInfo = TypedDict('MoveGripperParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveGripperParametersLocationCollisionInfosArrayElement = TypedDict('MoveGripperParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ExecuteRobotProgramParametersDynamicEnvironmentStateMapValue = TypedDict('ExecuteRobotProgramParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ExecuteRobotProgramParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ExecuteRobotProgramParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ExecuteRobotProgramParametersRobotBridgeConnectionInfo = TypedDict('ExecuteRobotProgramParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

ExecuteRobotProgramParametersLocationCollisionInfosArrayElement = TypedDict('ExecuteRobotProgramParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

SaveSceneParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SaveSceneParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SaveSceneParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SaveSceneParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SaveSceneParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SaveSceneParametersDynamicEnvironmentStateMapValue = TypedDict('SaveSceneParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SaveSceneParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SaveSceneParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SaveSceneParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SaveSceneParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SaveSceneReturns = dict[str, Any]

SaveGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SaveGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SaveGripperParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SaveGripperParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SaveGripperParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SaveGripperParametersDynamicEnvironmentStateMapValue = TypedDict('SaveGripperParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SaveGripperParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SaveGripperParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SaveGripperParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SaveGripperParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SaveGripperParametersRobotBridgeConnectionInfo = TypedDict('SaveGripperParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

SaveGripperParametersLocationCollisionInfosArrayElement = TypedDict('SaveGripperParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValue = TypedDict('MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveJointsToJointConfigurationStatesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveJointsToJointConfigurationStatesParametersRobotBridgeConnectionInfo = TypedDict('MoveJointsToJointConfigurationStatesParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveJointsToJointConfigurationStatesParametersLocationCollisionInfosArrayElement = TypedDict('MoveJointsToJointConfigurationStatesParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

MoveJointsToJointConfigurationStatesParametersGripperInfoVariantItemPrefix0 = dict[str, Any]

MoveJointsToJointConfigurationStatesParametersStartJointConfigurationStatesArrayElement = TypedDict('MoveJointsToJointConfigurationStatesParametersStartJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsToJointConfigurationStatesParametersGoalJointConfigurationStatesArrayElement = TypedDict('MoveJointsToJointConfigurationStatesParametersGoalJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveJointsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveJointsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveJointsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveJointsParametersDynamicEnvironmentStateMapValue = TypedDict('MoveJointsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveJointsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveJointsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveJointsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveJointsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveJointsParametersRobotBridgeConnectionInfo = TypedDict('MoveJointsParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveJointsParametersLocationCollisionInfosArrayElement = TypedDict('MoveJointsParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

MoveJointsParametersGripperInfoVariantItemPrefix0 = dict[str, Any]

MoveJointsParametersStartJointConfigurationStatesArrayElement = TypedDict('MoveJointsParametersStartJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsParametersGoalJointConfigurationStatesArrayElement = TypedDict('MoveJointsParametersGoalJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValue = TypedDict('MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveJointsToPositionConfigurationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveJointsToPositionConfigurationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveJointsToPositionConfigurationParametersRobotBridgeConnectionInfo = TypedDict('MoveJointsToPositionConfigurationParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveJointsToPositionConfigurationParametersLocationCollisionInfosArrayElement = TypedDict('MoveJointsToPositionConfigurationParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

MoveJointsToPositionConfigurationParametersGripperInfoVariantItemPrefix0 = dict[str, Any]

MoveJointsToPositionConfigurationParametersStartJointConfigurationStatesArrayElement = TypedDict('MoveJointsToPositionConfigurationParametersStartJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsToPositionConfigurationParametersGoalJointConfigurationStatesArrayElement = TypedDict('MoveJointsToPositionConfigurationParametersGoalJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveJointsToPositionConfigurationReturns = dict[str, Any]

StartMoveThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartMoveThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartMoveThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartMoveThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartMoveThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartMoveThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StartMoveThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartMoveThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartMoveThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartMoveThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartMoveThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StartMoveThreadParametersLocationCollisionInfosArrayElement = TypedDict('StartMoveThreadParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

StartMoveThreadParametersGripperInfoVariantItemPrefix0 = dict[str, Any]

StartMoveThreadParametersStartJointConfigurationStatesArrayElement = TypedDict('StartMoveThreadParametersStartJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartMoveThreadParametersGoalJointConfigurationStatesArrayElement = TypedDict('StartMoveThreadParametersGoalJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartMoveThreadParametersRobotBridgeConnectionInfo = TypedDict('StartMoveThreadParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValue = TypedDict('GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetRobotBridgeIOVariablesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetRobotBridgeIOVariablesParametersRobotBridgeConnectionInfo = TypedDict('GetRobotBridgeIOVariablesParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetRobotBridgeIOVariablesParametersLocationCollisionInfosArrayElement = TypedDict('GetRobotBridgeIOVariablesParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgeIOVariablesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgeIOVariablesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetRobotBridgeIOVariablesParametersRobotBridgeConnectionInfo = TypedDict('SetRobotBridgeIOVariablesParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

SetRobotBridgeIOVariablesParametersLocationCollisionInfosArrayElement = TypedDict('SetRobotBridgeIOVariablesParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeIkParamPositionParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeIkParamPositionParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeIkParamPositionParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeIkParamPositionParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ComputeIkParamPositionReturns = TypedDict('ComputeIkParamPositionReturns', {
    'translation': list[float],
    'quaternion': list[float],
    'direction': tuple[float, float, float],
    'angleXZ': float,
    'angleYX': float,
    'angleZY': float,
    'angleX': float,
    'angleY': float,
    'angleZ': float,
}, total=False)

ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeIKFromParametersParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeIKFromParametersParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeIKFromParametersParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeIKFromParametersParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

FacePickUpPriorities = TypedDict('FacePickUpPriorities', {
    'xp': int,
    'xn': int,
    'yp': int,
    'yn': int,
    'zp': int,
    'zn': int,
}, total=False)

RandomBoxInfo = TypedDict('RandomBoxInfo', {
    'allowedPlacementOrientations': int,
    'boxDirAngle': float,
    'toolOffsetDistancePriorityMult': float,
    'dictFacePriorities': FacePickUpPriorities,
    'generateCornerOffsets': bool,
    'objectType': str,
    'objectSize': tuple[float, float, float],
    'objectMass': float,
    'pickRegionMinSplitAxis': float,
    'randomBoxOrigin': tuple[float, float, float],
    'rollStepDegree': float,
    'scaleObjectWeightByTransferSpeedMult': bool,
    'sizePrecisionXYZ': tuple[float, float, float],
    'toolTranslationOffsets': list[tuple[float, float, float]],
    'usefaces': list[Literal['xp', 'yp', 'zp', 'xn', 'yn', 'zn']],
}, total=False)

ComputeIKFromParametersReturnsSolutionsArrayElement = dict[str, Any]

ComputeIKFromParametersReturnsErrorsArrayElement = dict[str, Any]

ComputeIKFromParametersReturns = TypedDict('ComputeIKFromParametersReturns', {
    'solutions': list[ComputeIKFromParametersReturnsSolutionsArrayElement],
    'errors': list[ComputeIKFromParametersReturnsErrorsArrayElement],
}, total=False)

ReloadModuleParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ReloadModuleParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ReloadModuleParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ReloadModuleParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ReloadModuleParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ReloadModuleParametersDynamicEnvironmentStateMapValue = TypedDict('ReloadModuleParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ReloadModuleParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ReloadModuleParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ReloadModuleParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ReloadModuleParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ReloadModuleParametersRobotBridgeConnectionInfo = TypedDict('ReloadModuleParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

ReloadModuleParametersLocationCollisionInfosArrayElement = TypedDict('ReloadModuleParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValue = TypedDict('ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ShutdownRobotBridgeParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ShutdownRobotBridgeParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ShutdownRobotBridgeParametersRobotBridgeConnectionInfo = TypedDict('ShutdownRobotBridgeParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

ShutdownRobotBridgeParametersLocationCollisionInfosArrayElement = TypedDict('ShutdownRobotBridgeParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetRobotBridgeStateParametersDynamicEnvironmentStateMapValue = TypedDict('GetRobotBridgeStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetRobotBridgeStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetRobotBridgeStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetRobotBridgeStateParametersRobotBridgeConnectionInfo = TypedDict('GetRobotBridgeStateParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetRobotBridgeStateParametersLocationCollisionInfosArrayElement = TypedDict('GetRobotBridgeStateParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValue = TypedDict('ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ClearRobotBridgeErrorParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ClearRobotBridgeErrorParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ClearRobotBridgeErrorParametersRobotBridgeConnectionInfo = TypedDict('ClearRobotBridgeErrorParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

ClearRobotBridgeErrorParametersLocationCollisionInfosArrayElement = TypedDict('ClearRobotBridgeErrorParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgePauseParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgePauseParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgePauseParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgePauseParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetRobotBridgePauseParametersRobotBridgeConnectionInfo = TypedDict('SetRobotBridgePauseParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

SetRobotBridgePauseParametersLocationCollisionInfosArrayElement = TypedDict('SetRobotBridgePauseParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgeResumeParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgeResumeParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetRobotBridgeResumeParametersRobotBridgeConnectionInfo = TypedDict('SetRobotBridgeResumeParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

SetRobotBridgeResumeParametersLocationCollisionInfosArrayElement = TypedDict('SetRobotBridgeResumeParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgeServoOnParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgeServoOnParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgeLockModeParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgeLockModeParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ResetSafetyFaultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResetSafetyFaultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResetSafetyFaultParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResetSafetyFaultParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResetSafetyFaultParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResetSafetyFaultParametersDynamicEnvironmentStateMapValue = TypedDict('ResetSafetyFaultParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResetSafetyFaultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResetSafetyFaultParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResetSafetyFaultParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResetSafetyFaultParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgeControlModeParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgeControlModeParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetDynamicObjectsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetDynamicObjectsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetDynamicObjectsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetDynamicObjectsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetDynamicObjectsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetDynamicObjectsParametersDynamicEnvironmentStateMapValue = TypedDict('GetDynamicObjectsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetDynamicObjectsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetDynamicObjectsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetDynamicObjectsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetDynamicObjectsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeRobotConfigsForGraspVisualizationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResetCacheTemplatesParametersDynamicEnvironmentStateMapValue = TypedDict('ResetCacheTemplatesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResetCacheTemplatesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResetCacheTemplatesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValue = TypedDict('SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetRobotBridgeExternalIOPublishingParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValue = TypedDict('RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RestoreSceneInitialStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RestoreSceneInitialStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValue = TypedDict('RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RunMotorControlTuningStepTestParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RunMotorControlTuningStepTestParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValue = TypedDict('RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RunMotorControlTuningMaximulLengthSequenceParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunMotorControlTuningMaximulLengthSequenceParametersRobotBridgeConnectionInfo = TypedDict('RunMotorControlTuningMaximulLengthSequenceParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

RunMotorControlTuningMaximulLengthSequenceParametersLocationCollisionInfosArrayElement = TypedDict('RunMotorControlTuningMaximulLengthSequenceParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValue = TypedDict('RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RunMotorControlTuningDecayingChirpParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunMotorControlTuningDecayingChirpParametersRobotBridgeConnectionInfo = TypedDict('RunMotorControlTuningDecayingChirpParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

RunMotorControlTuningDecayingChirpParametersLocationCollisionInfosArrayElement = TypedDict('RunMotorControlTuningDecayingChirpParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValue = TypedDict('RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RunMotorControlTuningGaussianImpulseParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunMotorControlTuningGaussianImpulseParametersRobotBridgeConnectionInfo = TypedDict('RunMotorControlTuningGaussianImpulseParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

RunMotorControlTuningGaussianImpulseParametersLocationCollisionInfosArrayElement = TypedDict('RunMotorControlTuningGaussianImpulseParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValue = TypedDict('RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RunMotorControlTuningBangBangResponseParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunMotorControlTuningBangBangResponseParametersRobotBridgeConnectionInfo = TypedDict('RunMotorControlTuningBangBangResponseParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

RunMotorControlTuningBangBangResponseParametersLocationCollisionInfosArrayElement = TypedDict('RunMotorControlTuningBangBangResponseParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValue = TypedDict('RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': RunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

RunDynamicsIdentificationTestParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

RunDynamicsIdentificationTestParametersRobotBridgeConnectionInfo = TypedDict('RunDynamicsIdentificationTestParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

RunDynamicsIdentificationTestParametersLocationCollisionInfosArrayElement = TypedDict('RunDynamicsIdentificationTestParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValue = TypedDict('GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetTimeToRunDynamicsIdentificationTestParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetTimeToRunDynamicsIdentificationTestParametersRobotBridgeConnectionInfo = TypedDict('GetTimeToRunDynamicsIdentificationTestParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetTimeToRunDynamicsIdentificationTestParametersLocationCollisionInfosArrayElement = TypedDict('GetTimeToRunDynamicsIdentificationTestParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValue = TypedDict('CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': CalculateTestRangeFromCollisionParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

CalculateTestRangeFromCollisionParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

CalculateTestRangeFromCollisionParametersRobotBridgeConnectionInfo = TypedDict('CalculateTestRangeFromCollisionParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

CalculateTestRangeFromCollisionParametersLocationCollisionInfosArrayElement = TypedDict('CalculateTestRangeFromCollisionParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValue = TypedDict('GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetMotorControlParameterSchemaParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetMotorControlParameterSchemaParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetMotorControlParameterParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetMotorControlParameterParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetMotorControlParameterParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetMotorControlParameterParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetMotorControlParameterParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetMotorControlParameterParametersDynamicEnvironmentStateMapValue = TypedDict('GetMotorControlParameterParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetMotorControlParameterParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetMotorControlParameterParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetMotorControlParameterParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetMotorControlParameterParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetMotorControlParameterParametersRobotBridgeConnectionInfo = TypedDict('GetMotorControlParameterParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetMotorControlParameterParametersLocationCollisionInfosArrayElement = TypedDict('GetMotorControlParameterParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

GetMotorControlParametersParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetMotorControlParametersParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetMotorControlParametersParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetMotorControlParametersParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetMotorControlParametersParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetMotorControlParametersParametersDynamicEnvironmentStateMapValue = TypedDict('GetMotorControlParametersParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetMotorControlParametersParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetMotorControlParametersParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetMotorControlParametersParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetMotorControlParametersParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetMotorControlParameterParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetMotorControlParameterParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetMotorControlParameterParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetMotorControlParameterParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetMotorControlParameterParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetMotorControlParameterParametersDynamicEnvironmentStateMapValue = TypedDict('SetMotorControlParameterParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetMotorControlParameterParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetMotorControlParameterParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetMotorControlParameterParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetMotorControlParameterParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetMotorControlParameterParametersRobotBridgeConnectionInfo = TypedDict('SetMotorControlParameterParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

SetMotorControlParameterParametersLocationCollisionInfosArrayElement = TypedDict('SetMotorControlParameterParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

IsProfilingRunningParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('IsProfilingRunningParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

IsProfilingRunningParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

IsProfilingRunningParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

IsProfilingRunningParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

IsProfilingRunningParametersDynamicEnvironmentStateMapValue = TypedDict('IsProfilingRunningParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[IsProfilingRunningParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': IsProfilingRunningParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': IsProfilingRunningParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

IsProfilingRunningParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StartProfilingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartProfilingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartProfilingParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartProfilingParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartProfilingParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartProfilingParametersDynamicEnvironmentStateMapValue = TypedDict('StartProfilingParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartProfilingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartProfilingParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartProfilingParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartProfilingParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StopProfilingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopProfilingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopProfilingParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopProfilingParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopProfilingParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopProfilingParametersDynamicEnvironmentStateMapValue = TypedDict('StopProfilingParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopProfilingParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopProfilingParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopProfilingParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopProfilingParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ReplaceBodiesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ReplaceBodiesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ReplaceBodiesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ReplaceBodiesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ReplaceBodiesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ReplaceBodiesParametersDynamicEnvironmentStateMapValue = TypedDict('ReplaceBodiesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ReplaceBodiesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ReplaceBodiesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ReplaceBodiesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ReplaceBodiesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ReplaceBodiesParametersReplaceInfosArrayElementContainerDynamicPropertiesMetaData = dict[str, Any]

CameraDynamicState = TypedDict('CameraDynamicState', {
    'imagetype': str,
    'sensorInWorldPose': list[list[float]],
    'sensorLinkName': str,
    'sensorName': str,
}, total=False)

Vector3 = tuple[float, float, float]

DetectionResultStateContentsAABBInContainer = TypedDict('DetectionResultStateContentsAABBInContainer', {
    'extents': Vector3,
    'pos': Vector3,
}, total=False)

DetectionResultStateMeasuredHeights = dict[str, float]

DetectionResultState = TypedDict('DetectionResultState', {
    'cameraDynamicStates': list[CameraDynamicState],
    'contentsAABBInContainer': DetectionResultStateContentsAABBInContainer,
    'detectionEndTimeStampMS': int,
    'detectionStartTimeStampMS': int,
    'endcapturetime': int,
    'enddetectiontime': int,
    'isBinDetectionOn': bool,
    'isContainerEmpty': int,
    'isContainerPresent': int,
    'maxCandidateSize': list[float],
    'maxCandidateSizeIMP': list[float],
    'measuredHeights': DetectionResultStateMeasuredHeights,
    'minCandidateSize': list[float],
    'minCandidateSizeIMP': list[float],
    'numDetectedParts': int,
    'resultInWorldFrame': bool,
    'slaverequestid': str,
    'startcapturetime': int,
    'startdetectiontime': int,
}, total=False)

ReplaceBodiesParametersReplaceInfosArrayElementContainerDynamicProperties = TypedDict('ReplaceBodiesParametersReplaceInfosArrayElementContainerDynamicProperties', {
    'containerId': str,
    'containerType': str,
    'containerUsage': str,
    'isContainerPresent': int,
    'isContainerEmpty': int,
    'isContainerEmptyOnArrival': int,
    'isPackFormationComplete': Union[int, bool],
    'numPlacedInContainer': int,
    'numItemsInPackFormation': int,
    'updateTimeStampMS': int,
    'sensorTimeStampMS': int,
    'metaData': ReplaceBodiesParametersReplaceInfosArrayElementContainerDynamicPropertiesMetaData,
    'detectionResultState': DetectionResultState,
}, total=False)

ReplaceBodiesParametersReplaceInfosArrayElement = TypedDict('ReplaceBodiesParametersReplaceInfosArrayElement', {
    'name': str,
    'uri': str,
    'pose': tuple[float, float, float, float, float, float, float],
    'containerDynamicProperties': ReplaceBodiesParametersReplaceInfosArrayElementContainerDynamicProperties,
}, total=False)

GetStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetStateParametersDynamicEnvironmentStateMapValue = TypedDict('GetStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetStateParametersRobotBridgeConnectionInfo = TypedDict('GetStateParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetStateParametersLocationCollisionInfosArrayElement = TypedDict('GetStateParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValue = TypedDict('EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': EnsureSyncWithRobotBridgeParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

EnsureSyncWithRobotBridgeParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValue = TypedDict('ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResetCachedRobotConfigurationStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResetCachedRobotConfigurationStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StopMoveThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopMoveThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopMoveThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopMoveThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopMoveThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopMoveThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StopMoveThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopMoveThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopMoveThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopMoveThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopMoveThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValue = TypedDict('SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetInstantaneousJointValuesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetInstantaneousJointValuesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValue = TypedDict('GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPackItemPoseInWorldParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPackItemPoseInWorldParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

TemplateTargetHashPropertiesSchema = TypedDict('TemplateTargetHashPropertiesSchema', {
    'useMaxPalletLayerNumber': bool,
    'useObjectPackingId': bool,
    'useObjectType': bool,
    'usePartType': bool,
    'useTargetMass': bool,
    'useTargetSize': bool,
    'useTargetKinematicsGeometryHash': bool,
}, total=False)

PackFormationParametersAutoPackFormationComputationParameters = TypedDict('PackFormationParametersAutoPackFormationComputationParameters', {
    'allowBestPackingPattern': bool,
    'maxPackItemSizeDeviation': tuple[float, float, float],
    'moveResultsToLastLayer': bool,
    'skipBestEffortSingleSKUPack': bool,
    'sizeEpsilonXYZ': tuple[float, float, float],
    'templateTargetHashProperties': TemplateTargetHashPropertiesSchema,
    'useCache': bool,
}, total=False)

GoalConfigurationParameters = TypedDict('GoalConfigurationParameters', {
    'addDestPriority': float,
    'allowFlipDown': bool,
}, total=False)

PointCloudProcessingParameters = TypedDict('PointCloudProcessingParameters', {
    'maxPointCloudNoise': float,
    'pointCloudFillMargin': float,
    'medianFilterWindowSize': float,
    'medianFilterMaxZOffsetToOverwrite': float,
    'overwritePlacementWithDynamicPointCloud': bool,
    'useDynamicPointCloud': bool,
}, total=False)

EnvironmentProcessingParameters = TypedDict('EnvironmentProcessingParameters', {
    'deltastepsize': float,
    'deltaHeight': float,
    'equalHeightThresh': float,
    'ignoreContainerWalls': bool,
    'paddingFullSizeMultiplier': float,
    'maxPartFullSizeDeviation': tuple[float, float, float],
    'pointCloudProcessingParameters': PointCloudProcessingParameters,
    'roundDownSize': float,
    'shrinkContainerInnerExtentsXY': tuple[float, float],
    'uncertaintyDepthPrecision': float,
    'invalidRegionWallHeight': float,
}, total=False)

LinearApproachParameters = TypedDict('LinearApproachParameters', {
    'destApproachOffsetByXY': float,
    'destApproachOffsetByZ': float,
    'destApproachPreferredFinalZOffset': float,
    'destApproachAngle': float,
    'destDropRatio': float,
    'dropOffsetZ': float,
    'dropOffsetZFirstLayer': float,
    'topEdgeMinIntersection': float,
    'tightDistanceThresh': float,
    'destApproachMode': Literal['default', 'highestEdge'],
    'approachFromAboveContentsMode': Literal['', 'All'],
    'intSafetyAngleRegionSizeWall': int,
    'intSafetyAngleRegionSizeObject': int,
    'approachLowestWallHeight': bool,
    'destApproachIgnoreObstacleHeight': float,
    'topApproachSurfaceClearance': float,
    'oppositeObstacleUncertaintyHeight': float,
    'destApproachForceFinalOffsets': list[tuple[float, float, float]],
    'intMaxPenetrationToApproachingTarget': int,
    'rejectUnsafeZApproach': bool,
    'enable2DirColumnApproachHack': bool,
    'enableWallSwitchingApproach': bool,
    'wallSwitchingApproachAngleBetweenFinalMovementAndFloor': float,
}, total=False)

LinearDepartParameters = TypedDict('LinearDepartParameters', {
    'destDepartOffsetByZ': float,
    'minDestDepartOffsetZ': float,
    'destDepartOffsetXYFromWall': float,
    'departToClearContainerWall': bool,
    'departToClearContents': bool,
    'departToClearMinHeight': float,
    'departToClearMinHeightAtFinal': float,
    'destDepartAccumApproachDirMult': float,
    'enableComputeDepartDirection': bool,
}, total=False)

PlacementConstraintParameters = TypedDict('PlacementConstraintParameters', {
    'alignToHighestWall': bool,
    'constraintPosition': tuple[float, float],
    'constraintRadius': float,
    'maxPalletHoleSizes': tuple[float, float],
    'intBoxSizeErosionFactor': int,
    'intOffsetFromWallRange': int,
    'intSupportingWallRange': int,
    'intGoalsPackingClearance': int,
    'intOppositeWallThresh': int,
    'intOppositeObjectThresh': int,
    'maxDistOfCOMAboveWallHeight': float,
    'maxNumDestApproachesToCompute': int,
    'maxPlacementHeightOffsetAboveWall': float,
    'maxTargetAboveWallHeight': float,
    'maxHeightOfTargetInContainer': float,
    'minSupportingAreaRatio': float,
    'minPureSupportingAreaRatio': float,
    'supportingWallEdgeRatio': float,
    'nearWallMinSupportingAreaRatio': float,
    'nearWallMinPureSupportingAreaRatio': float,
    'notFallRatioNearWall': float,
    'maxPlacementHeightOfTarget': float,
    'minEdgeWallZHeight': float,
    'jitterCOMRatioOffset': float,
    'jitterCOMRatioOffsetOpenWall': float,
    'supportingZRange': float,
    'supportingZRangeBottom': float,
    'wallHeightThresh': float,
}, total=False)

PlacementAreaByHeightInfo = TypedDict('PlacementAreaByHeightInfo', {
    'borderHeight': float,
    'minX': float,
    'maxX': float,
    'minY': float,
    'maxY': float,
}, total=False)

ToolCheckParameters = TypedDict('ToolCheckParameters', {
    'toolDepthErosionWindowSize': float,
    'toolDepthMinDeadRegion': float,
    'toolMaxOffsetFromCenter': tuple[float, float, float],
    'toolSizeSafetyPadding': float,
    'useLogVerboseToolDepthCheck': bool,
    'useToolDepthMatrixCheckCache': bool,
}, total=False)

PackFormationParametersPlacementValidationParameters = TypedDict('PackFormationParametersPlacementValidationParameters', {
    'linearApproachParameters': LinearApproachParameters,
    'linearDepartParameters': LinearDepartParameters,
    'placementConstraintParameters': PlacementConstraintParameters,
    'placementAreaByHeightInfos': list[PlacementAreaByHeightInfo],
    'toolCheckParameters': ToolCheckParameters,
}, total=False)

GroupComparatorParameters = TypedDict('GroupComparatorParameters', {
    'heightCoefficient': float,
    'longestCoefficient': float,
    'maxLoadPerVoxelCoefficient': float,
    'packingItemHeightDiscretizationStep': float,
    'similarLayerCoefficient': float,
    'use': bool,
    'weightCoefficient': float,
    'xySurfaceCoefficient': float,
}, total=False)

PackingCoefficients = TypedDict('PackingCoefficients', {
    'comCostCoefficient': float,
    'concavityPenalizationCoefficient': float,
    'floorEmptyCoefficient': float,
    'footprintBlowupFactor': float,
    'footprintPivotMult': float,
    'heuristicBlowupFactor': float,
    'innerPerimeterMult': float,
    'layerNotFilledCoefficient': float,
    'maxLoadLeftCoefficient': float,
    'notFilledVolumeCoefficient': float,
    'notSupportedAreaRatioCoefficient': float,
    'notTouchingRatioCoefficient': float,
    'numNotPackedLayersCoefficient': float,
    'numPlacedCoefficient': float,
    'numUniqueGroupsStartedCoefficient': float,
    'outerPerimeterMult': float,
    'packageDimensionCoefficients': tuple[float, float, float],
    'placedFilledAreaCoefficient': float,
    'placedFilledVolumeCoefficient': float,
    'sameOrientation2DCoefficient': float,
    'stabilityCoefficient': float,
    'stdHeightCoefficient': float,
    'touchingOppositeFacesCoefficient': float,
    'touchingRatioContainerEdgeCoefficient': float,
    'weightCoefficient': float,
}, total=False)

DynamicHeightAdjustmentParameters = TypedDict('DynamicHeightAdjustmentParameters', {
    'stopDynamicMaxPlacementHeightRatio': float,
    'numDynamicHeightAdjustLimit': int,
    'numStatesHeightLimitUpdateInterval': int,
    'minHeightIncrease': float,
    'use': bool,
}, total=False)

PackingSearchParameters = TypedDict('PackingSearchParameters', {
    'combineSameSizeIntoMetaBox': bool,
    'maxNumUniqueGroupStarted': int,
    'returnSolutionMode': Literal['AllPlaced', 'MaxSumAreaPlaced', 'MaxNumPlaced'],
    'returnFirstCandidateOn2DSearch': bool,
    'searchMode3D': Literal['groupFirst', 'layerFirst'],
    'stopAfterFirstSolution': bool,
    'outOfOrderSearch': bool,
    'timeoutSec': float,
    'solutionImproveTimeoutSec': float,
    'packingSpeedIntervalSec': float,
    'minItemsToPackInSpeedInterval': int,
    'maxNumItemsInPack': int,
    'sufficientFillingRate': float,
    'sufficientMassRatio': float,
    'badStatesPruningRatio': float,
    'numMaxSolutions2DSearch': int,
    'numMaxBestStatesPerRotation': int,
    'numMaxBestStatesPerFace': int,
    'maxStateBranchingFactor': int,
    'similarityIntersectionThresh': float,
    'dynamicHeightAdjustmentParameters': DynamicHeightAdjustmentParameters,
    'minNumGroupsToCheck': int,
    'minNumLayersToCheck': int,
    'forceCheckEachUniqueGroupMember': bool,
    'rejectNonSpreadFullLayers': bool,
    'useLayerPackingMode': bool,
    'pruneOnFullLayer': bool,
}, total=False)

PackingConstraintParameters = TypedDict('PackingConstraintParameters', {
    'packingForceLayerExtendWallDirection': int,
    'layerSupportingZRange': float,
    'packingPartMaxLoadMult': float,
    'packingOrderPriorityMask': int,
    'maxPackingOrderPriorityDistance': int,
    'placeOnlyNearSameItem': bool,
    'minToppleAngleOpenWallMult': float,
    'minToppleAngle': float,
    'minToppleAngleAtBottom': float,
    'minToppleAngleAtTop': float,
    'numLayersLessStable': int,
    'maxAllowedLayersInTower': int,
    'numMaxPlacementsUnder': int,
    'maxAllowedPlacementByRow': int,
    'maxAllowedPlacementByColumn': int,
    'maxTotalAllowedMass': float,
    'pivotCOM': tuple[float, float, float],
}, total=False)

PackFormationParametersPackingPlacementParameters = TypedDict('PackFormationParametersPackingPlacementParameters', {
    'edgeDetectorThresh': float,
    'extendLayerNearWallDistance': float,
    'placementOfBoxFromSurface': float,
    'use90DegreesRotations': Literal[1, 2, 3],
}, total=False)

PackingPostProcessingParameters = TypedDict('PackingPostProcessingParameters', {
    'centerFinalPackOnX': bool,
    'centerFinalPackOnY': bool,
    'maxSpreadPlacementsIteration': int,
    'maxSpreadPlacementGap': float,
    'forceCenterLayersByRow': bool,
    'forceCenterLayersByColumn': bool,
    'postProcessOrdering': bool,
    'spreadPackingPattern': bool,
}, total=False)

PackFormationParametersValidationParameters = TypedDict('PackFormationParametersValidationParameters', {
    'validationMaxAllowedItemPenetrationDistance': float,
    'validationPartTopPadding': float,
}, total=False)

Packing2DPatternCandidateFilterInfoMassRange = TypedDict('Packing2DPatternCandidateFilterInfoMassRange', {
    'lower': float,
    'upper': float,
}, total=False)

Packing2DPatternCandidateFilterInfoLongEdgeRange = TypedDict('Packing2DPatternCandidateFilterInfoLongEdgeRange', {
    'lower': float,
    'upper': float,
}, total=False)

Packing2DPatternCandidateFilterInfoShortEdgeRange = TypedDict('Packing2DPatternCandidateFilterInfoShortEdgeRange', {
    'lower': float,
    'upper': float,
}, total=False)

Packing2DPatternCandidateFilterInfoHeightRange = TypedDict('Packing2DPatternCandidateFilterInfoHeightRange', {
    'lower': float,
    'upper': float,
}, total=False)

Packing2DPatternCandidateFilterInfo = TypedDict('Packing2DPatternCandidateFilterInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'barcodes': list[str],
    'massRange': Packing2DPatternCandidateFilterInfoMassRange,
    'packingPatternName': str,
    'partTypePattern': str,
    'longEdgeRange': Packing2DPatternCandidateFilterInfoLongEdgeRange,
    'shortEdgeRange': Packing2DPatternCandidateFilterInfoShortEdgeRange,
    'heightRange': Packing2DPatternCandidateFilterInfoHeightRange,
}, total=False)

Packing2DPatternSelection = TypedDict('Packing2DPatternSelection', {
    'candidateInfos': list[Packing2DPatternCandidateFilterInfo],
    'use': bool,
    'useAutoPackWhenNoCandidate': bool,
}, total=False)

PackFormationParametersPatternGeneratorParameters = TypedDict('PackFormationParametersPatternGeneratorParameters', {
    'defaultReorderPlacementMode': Literal['none', 'NXNY', 'NXCY', 'NXPY', 'CXNY', 'CXCY', 'CXPY', 'PXNY', 'PXCY', 'PXPY', '-XNY', '-XCY', '-XPY', 'NX-Y', 'CX-Y', 'PX-Y', 'Default'],
    'defaultGapSize': float,
    'maxGapSizeAsShortestEdgeMult': float,
    'packing2DPatternAutoSelection': Packing2DPatternSelection,
}, total=False)

PlacementSearchPriorities = TypedDict('PlacementSearchPriorities', {
    'dynamicPackingMode': Literal['default', 'stacking'],
    'alignLongestFaceToXPriority': float,
    'biggestSquareSizeCoefficient': float,
    'depthCoefficient': float,
    'distanceFromSearchCornerCoefficient': float,
    'intBiggestSquareDiscretization': int,
    'leftoverMinContainerDimensionCoefficient': float,
    'minDistanceToWallsCoefficient': float,
    'wallTouchingCoefficient': float,
    'ratioTouchingWallsDiscretization': float,
    'sideWallClearanceForVerticalDownCoefficient': float,
    'supportedAreaScore': float,
    'use': bool,
}, total=False)

SinglePlacementParameters = TypedDict('SinglePlacementParameters', {
    'placementPriorityParameters': PlacementSearchPriorities,
    'maxPlacementsToCompute': int,
    'edgeDetectorThresh': float,
    'continueSearchTimeLimit': float,
    'pivotCheckPositionMode': Literal['none', 'NXNY', 'NXCY', 'NXPY', 'CXNY', 'CXCY', 'CXPY', 'PXNY', 'PXCY', 'PXPY', '-XNY', '-XCY', '-XPY', 'NX-Y', 'CX-Y', 'PX-Y'],
    'tiltAngle': float,
    'tiltExtentsThresh': float,
    'useTiltPlacement': bool,
    'use90DegreesRotations': Literal[1, 2, 3],
}, total=False)

PackFormationParametersCheckPlacementParameters = TypedDict('PackFormationParametersCheckPlacementParameters', {
    'applyAvoidOldGoalsNeighToFinalPlacement': bool,
    'intMaxPackageTranslationOffsetXY': tuple[int, int],
    'intPackingCheckGoalsOffset': int,
    'forcePushDistance': float,
    'maxAllowedPushDistanceForPointCloud': float,
    'maxAllowedHeightDeviation': float,
    'normalizePackToEmptyRegion': bool,
    'numPlacementsToConsider': int,
    'packFollowMode': Literal['none', 'strict', 'strictStopOnFailure', 'smallerFirst', 'smallerFirstThenStrict'],
    'preserveOriginalPose': bool,
    'sizeEpsilonXYZ': tuple[float, float, float],
    'supportingZRangeMult': float,
    'useSimilarPoses': bool,
}, total=False)

UnitInfo = TypedDict('UnitInfo', {
    'lengthUnit': Literal['m', 'dm', 'cm', 'mm', 'um', 'nm', 'in', 'ft'],
    'massUnit': Literal['g', 'mg', 'kg', 'lb'],
    'timeDurationUnit': Literal['s', 'ms', 'us', 'ns', 'ps'],
    'angleUnit': Literal['rad', 'deg'],
    'timeStampUnit': Literal['s', 'ms', 'us', 'iso8601'],
}, total=False)

PackFormationParameters = TypedDict('PackFormationParameters', {
    'acceptableContainerSize1D': int,
    'allowMultipleContainer': bool,
    'autoPackFormationComputationParameters': PackFormationParametersAutoPackFormationComputationParameters,
    'debuglevel': Literal[-1, 0, 1, 2, 3, 4, 5],
    'goalConfigurationParameters': GoalConfigurationParameters,
    'environmentProcessingParameters': EnvironmentProcessingParameters,
    'placementValidationParameters': PackFormationParametersPlacementValidationParameters,
    'numSimultaneousContainersToPack': int,
    'groupComparatorParameters': GroupComparatorParameters,
    'packingCoefficients': PackingCoefficients,
    'packingSearchParameters': PackingSearchParameters,
    'packingConstraintParameters': PackingConstraintParameters,
    'packingPlacementParameters': PackFormationParametersPackingPlacementParameters,
    'postProcessingParameters': PackingPostProcessingParameters,
    'isStrictGroupSizeOrdering': bool,
    'maxPalletLayerNumber': int,
    'validationParameters': PackFormationParametersValidationParameters,
    'packingPatternName': str,
    'patternGeneratorParameters': PackFormationParametersPatternGeneratorParameters,
    'singlePlacementParameters': SinglePlacementParameters,
    'checkPlacementParameters': PackFormationParametersCheckPlacementParameters,
    'unitInfo': UnitInfo,
}, total=False)

PackFormationItem = TypedDict('PackFormationItem', {
    'partType': str,
    'mass': float,
    'partFullSize': tuple[float, float, float],
    'partMaxLoad': float,
    'aabbPoseInContainer': tuple[float, float, float, float, float, float, float],
    'supportedAreaRatio': float,
    'fillRatio': float,
    'weight': float,
    'com': list[Any],
    'containerIdx': int,
    'containerInnerFullSizeZ': float,
    'poseInDest': Optional[tuple[float, float, float, float, float, float, float]],
    'noPlacementAbove': Optional[str],
    'partMaxLoadPerVoxel': float,
    'boxExtentsInContainer': tuple[float, float, float],
    'boxExtentsInContainerPadded': tuple[float, float, float],
    'iSimilarGroup': int,
    'iPackingItem': int,
    'iUniqueGroup': int,
    'objectPackingId': int,
    'objectType': str,
    'fragilityIndex': int,
    'packingOrderPriority': str,
    'vPackingItemIndices': list[int],
    'vLocalTransforms': list[tuple[float, float, float, float, float, float, float]],
    'facesWithLabel': int,
}, total=False)

PackFormation = TypedDict('PackFormation', {
    'containerInnerFullSize': tuple[float, float, float],
    'comPackedItems': tuple[float, float, float],
    'containerName': str,
    'costWholePack': float,
    'fillRatio': float,
    'finishTime': float,
    'startTime': float,
    'maxHeightOfTargetInContainer': float,
    'numInitialPacked': int,
    'packContainerType': str,
    'packageDimensions': tuple[float, float, float],
    'userPackFormationParameters': PackFormationParameters,
    'packingUniqueId': str,
    'packingPatternName': str,
    'timestamp': float,
    'unit': Literal['m', 'dm', 'cm', 'mm', 'um', 'nm', 'in', 'ft'],
    'unitLength': Literal['m', 'dm', 'cm', 'mm', 'um', 'nm', 'in', 'ft'],
    'unitMass': Literal['g', 'mg', 'kg', 'lb'],
    'vContainerInnerExtents': tuple[float, float, float],
    'vPackingItems': list[PackFormationItem],
}, total=False)

VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

VisualizePackFormationResultParametersDynamicEnvironmentStateMapValue = TypedDict('VisualizePackFormationResultParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': VisualizePackFormationResultParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

VisualizePackFormationResultParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

VisualizePackFormationResultParametersPackLocationInfo = TypedDict('VisualizePackFormationResultParametersPackLocationInfo', {
    'containerId': Optional[str],
    'containerType': Optional[str],
    'locationName': str,
}, total=False)

VisualizePackFormationResultParametersPackInputPartInfosArrayElement = dict[str, Any]

RandomPackingParameters = TypedDict('RandomPackingParameters', {
    'startSearchCorner': int,
    'intAvoidOldGoalsNeighDelta': int,
}, total=False)

VisualizePackFormationResultParametersDynamicGoalsGeneratorParametersUserPackFormationParameters = dict[str, Any]

VisualizePackFormationResultParametersDynamicGoalsGeneratorParameters = TypedDict('VisualizePackFormationResultParametersDynamicGoalsGeneratorParameters', {
    'allowFallbackToRandom': bool,
    'autoComputePackFormation': bool,
    'autoRotatePackFormation': bool,
    'randomPackingParameters': RandomPackingParameters,
    'skipPackFormationValidation': bool,
    'saveDynamicGoalGeneratorState': bool,
    'saveDynamicGoalGeneratorStateFailed': bool,
    'saveDynamicGoalGeneratorStateOnRandomMode': bool,
    'useComputePackFormationFromState': bool,
    'userPackFormationParameters': VisualizePackFormationResultParametersDynamicGoalsGeneratorParametersUserPackFormationParameters,
}, total=False)

ConstraintToolInfo = TypedDict('ConstraintToolInfo', {
    'cameraDirectionAngle': float,
    'destDirectionAngle': float,
    'duringGrabbingAngle': float,
    'duringFreeMovementAngle': float,
    'finalDirectionAngle': float,
    'globalDirection': tuple[float, float, float],
    'graspDirectionAngle': float,
    'graspNormalizeToZeroRatio': float,
    'localToolDirection': tuple[float, float, float],
    'localCameraToolDirection': tuple[float, float, float],
    'use': bool,
}, total=False)

DistanceMeasurementInfo = TypedDict('DistanceMeasurementInfo', {
    'use': bool,
    'objectName': str,
    'triggerIOName': str,
    'triggerOutputDelay': float,
    'triggerOutputDuration': float,
    'baseDist': float,
    'measureBeforeFlipping': bool,
    'triggerOutputDelayForReplan': float,
}, total=False)

VisualizePackFormationResultReturnsPackedItemsInfoArrayElement = TypedDict('VisualizePackFormationResultReturnsPackedItemsInfoArrayElement', {
    'globalPose': tuple[float, float, float, float, float, float, float],
    'localPose': tuple[float, float, float, float, float, float, float],
    'isVisualize': bool,
    'isPackItem': bool,
    'partType': str,
    'color': tuple[float, float, float],
    'fullSize': tuple[float, float, float],
    'name': str,
    'uri': str,
}, total=False)

VisualizePackFormationResultReturns = TypedDict('VisualizePackFormationResultReturns', {
    'packedItemsInfo': list[VisualizePackFormationResultReturnsPackedItemsInfoArrayElement],
}, total=False)

ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValue = TypedDict('ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ClearPackingStateVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ClearPackingStateVisualizationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]


