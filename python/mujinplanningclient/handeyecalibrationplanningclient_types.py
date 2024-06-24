
# This file contains the types used in the client
from typing import TypedDict, Union, Any, Optional, Literal
from typing_extensions import Required

ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeCalibrationPosesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeCalibrationPosesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SensorName = str

SensorLinkName = str

SensorSelectionInfo = TypedDict('SensorSelectionInfo', {
    'sensorName': SensorName,
    'sensorLinkName': SensorLinkName,
}, total=False)

ComputeCalibrationPosesParametersCalibboardvisibility = dict[str, Any]

SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValue = TypedDict('SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SampleCalibrationConfigurationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SampleCalibrationConfigurationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SampleCalibrationConfigurationParametersCalibboardvisibility = dict[str, Any]

SampleCalibrationConfigurationReturns = TypedDict('SampleCalibrationConfigurationReturns', {
    'vConfig': list[float],
}, total=False)

PingReturns = TypedDict('PingReturns', {
    'timestamp': float,
    'slaverequestid': str,
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

GetGrabbedReturns = TypedDict('GetGrabbedReturns', {
    'names': Optional[list[str]],
}, total=False)

GetTransformReturns = TypedDict('GetTransformReturns', {
    'translation': list[float],
    'rotationmat': list[list[float]],
    'quaternion': list[float],
}, total=False)

GetLinkParentInfoReturns = TypedDict('GetLinkParentInfoReturns', {
    'name': str,
    'translation': list[Any],
    'rotationmat': list[Any],
    'quaternion': list[Any],
}, total=False)

GetOBBReturns = TypedDict('GetOBBReturns', {
    'extents': Any,
    'boxLocalTranslation': Any,
    'originalBodyTranslation': Any,
    'quaternion': Any,
    'rotationmat': Any,
    'translation': Any,
}, total=False)

GetInnerEmptyRegionOBBReturns = TypedDict('GetInnerEmptyRegionOBBReturns', {
    'extents': Any,
    'boxLocalTranslation': Any,
    'originalBodyTranslation': Any,
    'quaternion': Any,
    'rotationmat': Any,
    'translation': Any,
}, total=False)

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

GetAABBReturns = TypedDict('GetAABBReturns', {
    'pos': list[Any],
    'extents': list[Any],
}, total=False)

GetLocationTrackingInfosReturns = dict[str, Any]

RemoveObjectsWithPrefixReturns = TypedDict('RemoveObjectsWithPrefixReturns', {
    'removedBodyNames': Any,
}, total=False)

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

SaveSceneReturns = dict[str, Any]

MoveJointsToPositionConfigurationReturns = dict[str, Any]

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

ComputeIKFromParametersReturnsSolutionsArrayElement = dict[str, Any]

ComputeIKFromParametersReturnsErrorsArrayElement = dict[str, Any]

ComputeIKFromParametersReturns = TypedDict('ComputeIKFromParametersReturns', {
    'solutions': list[ComputeIKFromParametersReturnsSolutionsArrayElement],
    'errors': list[ComputeIKFromParametersReturnsErrorsArrayElement],
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


