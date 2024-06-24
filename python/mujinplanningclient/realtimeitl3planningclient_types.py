
# This file contains the types used in the client
from typing import TypedDict, Union, Any, Optional, Literal
from typing_extensions import Required

SetJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetJointValuesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetJointValuesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetJointValuesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetJointValuesParametersDynamicEnvironmentStateMapValue = TypedDict('SetJointValuesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetJointValuesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetJointValuesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetJointValuesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetJointValuesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetITLStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetITLStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetITLStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetITLStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetITLStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetITLStateParametersDynamicEnvironmentStateMapValue = TypedDict('GetITLStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetITLStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetITLStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetITLStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetITLStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetITLStateParametersRobotBridgeConnectionInfo = TypedDict('GetITLStateParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetITLStateParametersLocationCollisionInfosArrayElement = TypedDict('GetITLStateParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ExecuteTrajectoryParametersDynamicEnvironmentStateMapValue = TypedDict('ExecuteTrajectoryParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ExecuteTrajectoryParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ExecuteTrajectoryParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValue = TypedDict('ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ExecuteTrajectoryStepParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ExecuteTrajectoryStepParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValue = TypedDict('PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PauseExecuteTrajectoryParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PauseExecuteTrajectoryParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValue = TypedDict('ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResumeExecuteTrajectoryParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResumeExecuteTrajectoryParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

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

ComputeRobotConfigsForCommandVisualizationParametersExecutiongraphCommandsArrayElement = TypedDict('ComputeRobotConfigsForCommandVisualizationParametersExecutiongraphCommandsArrayElement', {
    'ikParam': list[IkParameterization],
}, total=False)

ComputeRobotConfigsForCommandVisualizationParametersExecutiongraph = TypedDict('ComputeRobotConfigsForCommandVisualizationParametersExecutiongraph', {
    'commands': list[ComputeRobotConfigsForCommandVisualizationParametersExecutiongraphCommandsArrayElement],
}, total=False)

ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PlotProgramWaypointsParametersDynamicEnvironmentStateMapValue = TypedDict('PlotProgramWaypointsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PlotProgramWaypointsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PlotProgramWaypointsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StartITLProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartITLProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartITLProgramParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartITLProgramParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartITLProgramParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartITLProgramParametersDynamicEnvironmentStateMapValue = TypedDict('StartITLProgramParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartITLProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartITLProgramParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartITLProgramParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartITLProgramParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StartITLProgramParametersRobotBridgeConnectionInfo = TypedDict('StartITLProgramParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

StartITLProgramParametersLocationCollisionInfosArrayElement = TypedDict('StartITLProgramParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
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

StartITLProgramParametersToolposesCycleStart = TypedDict('StartITLProgramParametersToolposesCycleStart', {
    'ikparamname': str,
    'toolname': str,
    'useSourceContainerIkParams': bool,
}, total=False)

StartITLProgramParametersToolposes = TypedDict('StartITLProgramParametersToolposes', {
    'cycleStart': StartITLProgramParametersToolposesCycleStart,
}, total=False)

StartITLProgramParametersDefaultItlProgramParams = TypedDict('StartITLProgramParametersDefaultItlProgramParams', {
    'cnt': bool,
    'unit': Literal['m', 'cm', 'mm', 'um', 'nm', 'inch'],
    'toolAccelTrans': float,
    'maxJointSpeedRatio': float,
    'workspaceDistThresh': float,
    'constraintDirThresh': tuple[float, float, float, float, float, float, float],
    'checkToolCollision': bool,
    'optimizationValue': float,
    'ignorePosture': bool,
    'envClearance': Optional[float],
}, total=False)

StartITLProgramParametersParameters = dict[str, Any]

StopITLProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopITLProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopITLProgramParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopITLProgramParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopITLProgramParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopITLProgramParametersDynamicEnvironmentStateMapValue = TypedDict('StopITLProgramParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopITLProgramParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopITLProgramParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopITLProgramParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopITLProgramParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GenerateExecutionGraphParametersDynamicEnvironmentStateMapValue = TypedDict('GenerateExecutionGraphParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GenerateExecutionGraphParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GenerateExecutionGraphParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GenerateExecutionGraphParametersRobotBridgeConnectionInfo = TypedDict('GenerateExecutionGraphParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GenerateExecutionGraphParametersLocationCollisionInfosArrayElement = TypedDict('GenerateExecutionGraphParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

GenerateExecutionGraphParametersParameters = dict[str, Any]

GenerateExecutionGraphReturnsCommandsArrayElement = dict[str, Any]

GenerateExecutionGraphReturnsProgramErrorVariantItemPrefix0 = dict[str, Any]

GenerateExecutionGraphReturns = TypedDict('GenerateExecutionGraphReturns', {
    'commands': list[GenerateExecutionGraphReturnsCommandsArrayElement],
    'programError': Optional[GenerateExecutionGraphReturnsProgramErrorVariantItemPrefix0],
    'manipForTrajs': list[list[str]],
}, total=False)

PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PopulateTargetInContainerParametersDynamicEnvironmentStateMapValue = TypedDict('PopulateTargetInContainerParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PopulateTargetInContainerParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PopulateTargetInContainerParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

PopulateTargetInContainerParametersContainerMetaData = dict[str, Any]

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


