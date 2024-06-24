
# This file contains the types used in the client
from typing import TypedDict, Union, Any, Optional, Literal
from typing_extensions import Required

PickAndPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PickAndPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PickAndPlaceParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PickAndPlaceParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PickAndPlaceParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PickAndPlaceParametersDynamicEnvironmentStateMapValue = TypedDict('PickAndPlaceParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PickAndPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PickAndPlaceParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PickAndPlaceParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PickAndPlaceParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartPickAndPlaceThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartPickAndPlaceThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ApproachGraspOffset = float

ScannerCameraInfo = TypedDict('ScannerCameraInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'name': str,
    'scanFieldUsageRatio': float,
    'scannerDist': float,
    'scanTargetBodiesIO': str,
    'statusIO': str,
    'triggerIO': str,
}, total=False)

BarcodeScanningInfo = TypedDict('BarcodeScanningInfo', {
    'use': bool,
    'useCheckBarCodeMatchPattern': bool,
    'barCodeScannerAngleOfIncidence': float,
    'barcodeCheckPatternIOName': str,
    'barcodeCheckPatternBase': str,
    'barcodeExtendRegion': float,
    'checkCameraOcclusionAtScanBarcodes': bool,
    'checkExpectedDetectedHeightThreshold': float,
    'defaultScanningPattern': Literal['simplePassthrough', 'rotatePassthrough'],
    'forceScanningPattern': Literal['simplePassthrough', 'rotatePassthrough'],
    'numBarCodeRotations': int,
    'scannerCameras': list[ScannerCameraInfo],
    'scannerBarCodeChangeMaxDuration': float,
    'scannerPassthroughSpeed': float,
    'scannerSignalOutputDelay': float,
    'scannerSignalOutputDuration': float,
    'stopOnBarcodeError': bool,
    'timeBufferForStopping': float,
    'maxNumConsecutiveBarcodeScanFailures': int,
    'waitAtScanTime': float,
}, total=False)

MergeTrajectoryParameters = TypedDict('MergeTrajectoryParameters', {
    'badTrajTimeThresh': float,
    'linearTrajSegmentEndTime': float,
    'linearTrajJointThresh': float,
    'minJointThresh': float,
    'maxJointThresh': float,
    'singularityJointMult': float,
    'singularityJointThresh': float,
    'toolOrientationThresh': float,
    'toolTranslationThresh': float,
}, total=False)

PartPresenceIOSensorsDefinition = TypedDict('PartPresenceIOSensorsDefinition', {
    'id': str,
    'deleted': bool,
    'index': int,
    'ikparamNames': list[str],
    'ioname': str,
}, total=False)

BottomScanPlacementInfo = TypedDict('BottomScanPlacementInfo', {
    'allowedPlacementCenteringDistanceTowardSensor': float,
    'distanceConsistencyCheckThreshold': float,
    'distanceIOName': str,
    'enableScanSegmentSkip': bool,
    'intermediateSpeedChangePointOffsetForAccurateStop': int,
    'intermediateSpeedForAccurateStop': float,
    'invalidIOName': str,
    'loweringMarginTime': float,
    'maxNumConsecutiveDistanceInconsistency': int,
    'maxNumPartPresenceDetectedDuringOnePick': int,
    'minNumMeasurements': int,
    'measurableAreaOffsetFromEdge': float,
    'measurableAreaRatio': float,
    'mergeTrajectoryParameters': MergeTrajectoryParameters,
    'minNumMeasurementsRatio': float,
    'multipleSensorsDifferenceThreshold': float,
    'outlierSignificanceThreshold': float,
    'overRangeTolerance': float,
    'partPresenceCheckTimeout': float,
    'partPresenceIOSensors': list[PartPresenceIOSensorsDefinition],
    'pauseIfPartPresent': bool,
    'preScanMotionLength': float,
    'scanDirection': tuple[float, float, float],
    'shorterEdgeLengthThresholdToScanFullWidth': float,
    'scanLength': float,
    'scanSpeedMult': float,
    'sensorFrequency': float,
    'sensorIkparamName': str,
    'signalScale': float,
    'standardDeviationThreshold': float,
    'timestampIOName': str,
    'triggerOutputDelay': float,
    'use': bool,
    'validIOName': str,
}, total=False)

CameraPlanningInfo = TypedDict('CameraPlanningInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'additionalIkParamNames': list[str],
    'adjustIkParamZUsingHighestItem': bool,
    'cameraManipName': str,
    'checkContainerValidTimeStamp': bool,
    'checkGraspVisibility': bool,
    'clearanceXYForGraspVisibilityFilter': float,
    'clearanceZForGraspVisibilityFilter': float,
    'graspsetname': str,
    'ikParamName': str,
    'instobjectname': str,
    'triggerLocationNames': list[str],
    'planningTriggerTiming': Literal['inCyclePerTarget', 'inCyclePerContainer', 'initialBeforePlanning', 'onPlanningIdle'],
    'preshapeFingers': Optional[list[float]],
    'replanningTimeLimit': float,
    'saveNoIKScene': bool,
    'saveUpdatedScene': bool,
    'timeoutWaitForFineDetectionResult': float,
    'timeoutWaitForTrigger': float,
    'detectionTriggerType': Literal['detection', 'phase1Detection', 'phase2Detection', 'pointCloudObstacle'],
    'maxPickupNumOverlappingPoints': int,
    'mergeTrajectoryParameters': MergeTrajectoryParameters,
    'robotSpeedMult': float,
    'use': bool,
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

CycleOverwriteParametersDestApproachInfosSegmentsArrayElement = TypedDict('CycleOverwriteParametersDestApproachInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

CycleOverwriteParametersDestApproachInfos = TypedDict('CycleOverwriteParametersDestApproachInfos', {
    'segments': list[CycleOverwriteParametersDestApproachInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

CycleOverwriteParametersDestDepartInfosSegmentsArrayElement = TypedDict('CycleOverwriteParametersDestDepartInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

CycleOverwriteParametersDestDepartInfos = TypedDict('CycleOverwriteParametersDestDepartInfos', {
    'segments': list[CycleOverwriteParametersDestDepartInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

AllDestCoordType = Literal['tool', 'target', 'toolzntarget', 'targetbottom', 'targetplaceik', 'targetAnyBottomFace', 'targetAnyBottomFaceAlignedX', 'targetAnyBottomFaceAlignedY', 'targetAnyBottomFaceXIsLongAxis', 'targetCorner', 'targetbottom_py_InductionCV', 'targetStack', 'targetAnyBottomBarCodeFace', 'targetAABBAlignIk']

DestTargetAnyBottomFaceRotationParameters = TypedDict('DestTargetAnyBottomFaceRotationParameters', {
    'destDepartExtraOffsetDirAfterTilt': tuple[float, float, float],
    'destTranslationOffsetOnTilt': tuple[float, float, float],
    'extraTravelDist': float,
    'facePlaceEdge': tuple[float, float, float],
    'minTargetLengthAlongConveyorDist': float,
    'penaltyOnLowerPriorityTiltAngle': float,
    'tiltAngles': list[float],
    'tiltAnglesForLightweightTarget': list[float],
    'tiltAnglesForLightweightTargetMassThresh': float,
    'tiltAnglesForLightweightTargetThreshold': float,
    'tiltExtentsThresh': float,
    'tiltExtentsRatioThresh': float,
    'isTiltAllowedOnGrabbingZSurface': bool,
    'tiltTargetOnGrabbingSmallestSurface': bool,
    'tiltShortSide': Literal[-1, 0, 1],
    'tiltLongSide': Literal[-1, 0, 1],
    'tiltTranslationDistanceMult': float,
    'rotationHeadingXAxis': Literal['None', 'PX', 'NX', 'PY', 'NY'],
    'useUpAxisFlip': Literal[-1, 0, 1],
}, total=False)

DestTargetCornerParameters = TypedDict('DestTargetCornerParameters', {
    'travelDist': float,
    'destFilterByTargetCornerDistThresh': float,
}, total=False)

TriggerDecelIOSensorDefinition = TypedDict('TriggerDecelIOSensorDefinition', {
    'id': str,
    'deleted': bool,
    'index': int,
    'action': Literal['decel', 'stop'],
    'ikparamNames': list[str],
    'ioname': str,
    'rayCheckMode': Literal['all', 'any'],
}, total=False)

DropInDestInfo = TypedDict('DropInDestInfo', {
    'expectedIOValueWhenBlocking': bool,
    'atEndIOCheck': bool,
    'extraHeightUncertaintyOnEndIOCheckFailure': float,
    'destApproachSpeedMultOnEndIOCheckFailure': float,
    'ioSensors': list[TriggerDecelIOSensorDefinition],
    'decelMultAfterIOMatches': float,
    'decelSpeedAfterIOMatches': float,
    'pauseIfStopIOInitiallyON': bool,
    'pauseTimeOutDuration': float,
    'regionBodyNames': list[str],
    'mergeTrajectoryParameters': MergeTrajectoryParameters,
    'use': bool,
}, total=False)

ExecutionVerificationInfoObjectTypeMap = dict[str, Any]

ExecutionVerificationInfo = TypedDict('ExecutionVerificationInfo', {
    'bottomThicknessForPushVerification': float,
    'bottomOverhangForPushVerificationXYZ': tuple[float, float, float],
    'destAfterChangeDuration': float,
    'destApproachIgnoreDistance': float,
    'destApproachIgnoreDistanceXY': float,
    'destApproachIgnoreDistanceZ': float,
    'pointCloudProjectedDist': float,
    'pointSizeMult': float,
    'sourceAfterChangeDuration': float,
    'sourceApproachIgnoreDistance': float,
    'stopOnLatePointCloud': bool,
    'use': bool,
    'verifyDestMode': Literal['never', 'lastDetection', 'pointCloudOnChange', 'pointCloudOnChangeAfterGrab', 'pointCloudAlways', 'pointCloudOnChangeFirstCycleOnly', 'pointCloudOnChangeWithDuration'],
    'verifySourceMode': Literal['never', 'lastDetection', 'pointCloudOnChange', 'pointCloudAlways', 'pointCloudOnChangeFirstCycleOnly', 'pointCloudOnChangeAfterGrab'],
    'verifyTargetPositionThresh': float,
    'objectTypeMap': ExecutionVerificationInfoObjectTypeMap,
}, total=False)

CollisionWallOffsetParameters = TypedDict('CollisionWallOffsetParameters', {
    'stepLengthXY': float,
    'stepLengthZ': float,
    'clearanceMult': float,
    'allowFallback': bool,
    'use': bool,
}, total=False)

CycleOverwriteParametersGraspApproachInfosSegmentsArrayElement = TypedDict('CycleOverwriteParametersGraspApproachInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

CycleOverwriteParametersGraspApproachInfos = TypedDict('CycleOverwriteParametersGraspApproachInfos', {
    'segments': list[CycleOverwriteParametersGraspApproachInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

CycleOverwriteParametersGraspDepartInfosSegmentsArrayElement = TypedDict('CycleOverwriteParametersGraspDepartInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

CycleOverwriteParametersGraspDepartInfos = TypedDict('CycleOverwriteParametersGraspDepartInfos', {
    'segments': list[CycleOverwriteParametersGraspDepartInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

ObjectMassPropertiesThresholds = TypedDict('ObjectMassPropertiesThresholds', {
    'id': str,
    'deleted': bool,
    'index': int,
    'centerOfMassThreshold': float,
    'massLowerThreshold': float,
    'massUpperThreshold': float,
    'massLowerThresholdMult': float,
    'massUpperThresholdMult': float,
    'massLowerThresholdMin': float,
    'massUpperThresholdMin': float,
    'areaDensityLowerThreshold': float,
    'areaDensityUpperThreshold': float,
}, total=False)

ObjectMassPropertiesCheckingInfo = TypedDict('ObjectMassPropertiesCheckingInfo', {
    'allowedForceStandardDeviationOnRemeasurement': list[float],
    'canRemeasureAndContinue': bool,
    'checkDuringGraspDepart': bool,
    'checkDuringDestApproach': bool,
    'disableRemeasureCenterOfMass': bool,
    'forceTorqueSensorLinkName': str,
    'indexThresholdsForDestApproach': int,
    'indexThresholdsForGraspDepart': int,
    'maxNumRemeasurementOnStandardDeviationFailure': int,
    'overwriteExpectedCenterOfMassWithEstimatedOnGraspDepartEnd': bool,
    'overwriteExpectedMassWithEstimatedOnGraspDepartEnd': bool,
    'remeasurementThresholds': ObjectMassPropertiesThresholds,
    'remeasureWithNearCalibrationPosition': bool,
    'remeasureWithSensorHorizontalPosition': bool,
    'shiftDistanceOnLateralForce': float,
    'thresholdsList': list[ObjectMassPropertiesThresholds],
    'triggerIOName': str,
    'triggerOutputDelay': float,
    'triggerOutputDuration': float,
    'triggerStartHeight': float,
    'use': bool,
}, total=False)

PlacedTargetRemainingTranslationRange = tuple[float, float, float]

CycleOverwriteParameters = TypedDict('CycleOverwriteParameters', {
    'approachoffset': ApproachGraspOffset,
    'approachCurrentExceedThresholds': list[float],
    'approachCurrentExceedThresholdsDelta': list[float],
    'approachForceTorqueExceedThresholds': list[float],
    'approachForceTorqueExceedThresholdsDelta': list[float],
    'barcodeScanningInfo': BarcodeScanningInfo,
    'bottomScanPlacementInfo': BottomScanPlacementInfo,
    'constraintToolInfo': ConstraintToolInfo,
    'destApproachInfos': CycleOverwriteParametersDestApproachInfos,
    'destDepartInfos': CycleOverwriteParametersDestDepartInfos,
    'destcoordtype': AllDestCoordType,
    'destApproachCurrentExceedThresholds': list[float],
    'destApproachCurrentExceedThresholdsDelta': list[float],
    'destApproachForceTorqueExceedThresholds': list[float],
    'destApproachForceTorqueExceedThresholdsDelta': list[float],
    'destFilterByTargetCornerDistThresh': float,
    'destFilterByTargetOrientationThresh': float,
    'destTargetAnyBottomFaceRotationParameters': DestTargetAnyBottomFaceRotationParameters,
    'destTargetCornerParameters': DestTargetCornerParameters,
    'doAutoRecoveryOnPieceLost': bool,
    'dropInDestInfo': DropInDestInfo,
    'executionVerificationInfo': ExecutionVerificationInfo,
    'finalPlanMode': Literal['none', '', 'cameraocclusion', 'config', 'configIgnoreOcclusion'],
    'graspApproachCollisionWallOffsetParameters': CollisionWallOffsetParameters,
    'graspDepartAboveNearbyObstacles': bool,
    'graspDepartCollisionWallOffsetParameters': CollisionWallOffsetParameters,
    'graspDepartCurrentExceedThresholds': list[float],
    'graspDepartCurrentExceedThresholdsDelta': list[float],
    'graspDepartForceTorqueExceedThresholds': list[float],
    'graspDepartForceTorqueExceedThresholdsDelta': list[float],
    'graspApproachInfos': CycleOverwriteParametersGraspApproachInfos,
    'graspDepartInfos': CycleOverwriteParametersGraspDepartInfos,
    'graspFilterByApproachOrientationThresh': float,
    'maxAllowedTargetSize': tuple[float, float, float],
    'maxAllowedTargetSizeObjectName': str,
    'maxGrabbingManipAccel': float,
    'maxGrabbingManipSpeed': float,
    'maxFreeManipAccel': float,
    'maxFreeManipSpeed': float,
    'moveToScanBarCodes': bool,
    'objectMassPropertiesCheckingInfo': ObjectMassPropertiesCheckingInfo,
    'placedTargetExtraPadding': tuple[float, float, float],
    'placedTargetExtraPaddingMinMax': tuple[float, float, float, float, float, float],
    'placedTargetRemainingTranslationRange': PlacedTargetRemainingTranslationRange,
    'targetPostActionAfterPick': Literal['setUnpickable', 'moveDownSetUnpickable', 'delete', 'deleteButConsiderForApproachMove', 'deleteNotifyBridge'],
    'testWallStepLengthXY': CollisionWallOffsetParameters,
    'toolSpeedAccelOptions': int,
    'useDynamicGoals': bool,
    'updateHeightWithMeasuredValue': bool,
    'verifyTargetPositionThresh': float,
    'graspsetname': str,
}, total=False)

IntermediateCycleInfo = TypedDict('IntermediateCycleInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'destikparamnames': list[str],
    'delayBeforeExecute': float,
    'targetPaddingBuffer': float,
    'preCycleOverwrite': CycleOverwriteParameters,
    'intermediateCycleOverwrite': CycleOverwriteParameters,
    'intermediateDestNames': list[str],
    'moveToFinishPositionAtEnd': bool,
    'onlyOnChangeAllowPlacement': bool,
    'overwriteDest': bool,
    'planToDest': bool,
    'validGraspSetName': str,
    'validTargetUri': str,
    'use': bool,
}, total=False)

DestBarcodeScanningInfo = TypedDict('DestBarcodeScanningInfo', {
    'use': bool,
    'useCheckBarCodeMatchPattern': bool,
    'useSamplingBasedPlanningForRotation': bool,
    'barcodeCheckPatternIOName': str,
    'barcodeCheckPatternBase': str,
    'expectedBarcodes': list[str],
    'totalNumBarCodes': int,
    'triggerIONames': list[str],
    'scannerSignalOutputDelay': float,
    'scannerSignalOutputDuration': float,
    'scannerSignalOutputInterval': float,
    'scannerBarCodeChangeMaxDuration': float,
    'skipWhenAllRegistered': bool,
    'barcodeScanningGain': float,
    'minNumBarCodes': int,
    'maxNumBarCodes': int,
    'updateBarcodeRegistration': bool,
    'barcodeInfoIOName': str,
    'liftingUpOffsetForRotation': tuple[float, float, float],
    'registerOppositeFace': bool,
    'scanMinViableRegionOnly': bool,
    'skipShorterFaceScanning': bool,
    'forcePlanWithNewBarcodeFace': bool,
    'destApproachAngularSpeed': float,
    'destApproachAngularAccel': float,
    'destApproachAccel': float,
    'destApproachSpeed': float,
    'destDepartAngularSpeed': float,
    'destDepartAngularAccel': float,
    'destDepartAccel': float,
    'destDepartSpeed': float,
    'additionalBarcodeReader': Literal['none', 'oppositeSide'],
    'returnToInitialPlacementOnNoRead': bool,
}, total=False)

StartPickAndPlaceThreadParametersDestBarcodeScanningInfoPerContainer = TypedDict('StartPickAndPlaceThreadParametersDestBarcodeScanningInfoPerContainer', {
    'locationName': str,
    'destBarcodeScanningInfo': DestBarcodeScanningInfo,
}, total=False)

DestTargetAABBAlignIkParameters = TypedDict('DestTargetAABBAlignIkParameters', {
    'boxLocalOffset': tuple[Literal[-1, 0, 1], Literal[-1, 0, 1], Literal[-1, 0, 1]],
    'quatLocalOffset': tuple[float, float, float, float],
    'globalUpAxisInPlace': Literal['x', 'y', 'z'],
    'alignLongAxisToX': bool,
    'alignLongAxisToY': bool,
    'alignLongAxisToZ': bool,
    'dimensionThreshold': float,
}, total=False)

DestTargetStackParameters = TypedDict('DestTargetStackParameters', {
    'destApproachOffsetByXY': float,
    'destApproachOffsetByZ': float,
    'destApproachRotationAngle': float,
    'offsetDist': float,
    'destHeightPenaltyMult': float,
    'stackHeightLimit': float,
}, total=False)

RandomPackingParameters = TypedDict('RandomPackingParameters', {
    'startSearchCorner': int,
    'intAvoidOldGoalsNeighDelta': int,
}, total=False)

StartPickAndPlaceThreadParametersDynamicGoalsParametersUserPackFormationParameters = dict[str, Any]

StartPickAndPlaceThreadParametersDynamicGoalsParameters = TypedDict('StartPickAndPlaceThreadParametersDynamicGoalsParameters', {
    'allowFallbackToRandom': bool,
    'autoComputePackFormation': bool,
    'autoRotatePackFormation': bool,
    'randomPackingParameters': RandomPackingParameters,
    'skipPackFormationValidation': bool,
    'saveDynamicGoalGeneratorState': bool,
    'saveDynamicGoalGeneratorStateFailed': bool,
    'saveDynamicGoalGeneratorStateOnRandomMode': bool,
    'useComputePackFormationFromState': bool,
    'userPackFormationParameters': StartPickAndPlaceThreadParametersDynamicGoalsParametersUserPackFormationParameters,
}, total=False)

StartPickAndPlaceThreadParametersGraspApproachInfosSegmentsArrayElement = TypedDict('StartPickAndPlaceThreadParametersGraspApproachInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

StartPickAndPlaceThreadParametersGraspApproachInfos = TypedDict('StartPickAndPlaceThreadParametersGraspApproachInfos', {
    'segments': list[StartPickAndPlaceThreadParametersGraspApproachInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

StartPickAndPlaceThreadParametersGraspApproachInfosPerURIMapValueSegmentsArrayElement = TypedDict('StartPickAndPlaceThreadParametersGraspApproachInfosPerURIMapValueSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

StartPickAndPlaceThreadParametersGraspApproachInfosPerURIMapValue = TypedDict('StartPickAndPlaceThreadParametersGraspApproachInfosPerURIMapValue', {
    'segments': list[StartPickAndPlaceThreadParametersGraspApproachInfosPerURIMapValueSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

StartPickAndPlaceThreadParametersGraspApproachInfosPerURI = dict[str, dict[str, Any]]

StartPickAndPlaceThreadParametersGraspDepartInfosSegmentsArrayElement = TypedDict('StartPickAndPlaceThreadParametersGraspDepartInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

StartPickAndPlaceThreadParametersGraspDepartInfos = TypedDict('StartPickAndPlaceThreadParametersGraspDepartInfos', {
    'segments': list[StartPickAndPlaceThreadParametersGraspDepartInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

StartPickAndPlaceThreadParametersDestApproachInfosSegmentsArrayElement = TypedDict('StartPickAndPlaceThreadParametersDestApproachInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

StartPickAndPlaceThreadParametersDestApproachInfos = TypedDict('StartPickAndPlaceThreadParametersDestApproachInfos', {
    'segments': list[StartPickAndPlaceThreadParametersDestApproachInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

DestApproachAccelDecelScaleMultOnTargetMass = TypedDict('DestApproachAccelDecelScaleMultOnTargetMass', {
    'id': str,
    'deleted': bool,
    'index': int,
    'scale': float,
    'mass': float,
}, total=False)

StartPickAndPlaceThreadParametersDestDepartInfosSegmentsArrayElement = TypedDict('StartPickAndPlaceThreadParametersDestDepartInfosSegmentsArrayElement', {
    'translationOffset': tuple[float, float, float],
    'quaternionOffset': tuple[float, float, float, float],
    'transSpeed': float,
    'transAccel': float,
    'transDecel': float,
    'rotationSpeed': float,
    'rotationAccel': float,
    'rotationDecel': float,
    'fingerValuesBefore': list[float],
    'fingerValuesAfter': list[float],
    'usePreviousFingerValuesIndices': list[int],
    'extendWhenHasContainerLid': float,
    'extendWhenNoContainerLid': float,
    'stopIOName': str,
    'stopSensorIsOn': bool,
}, total=False)

StartPickAndPlaceThreadParametersDestDepartInfos = TypedDict('StartPickAndPlaceThreadParametersDestDepartInfos', {
    'segments': list[StartPickAndPlaceThreadParametersDestDepartInfosSegmentsArrayElement],
    'offsetRefType': Literal['world', 'tool'],
}, total=False)

StartPickAndPlaceThreadParametersDestFilterByTargetOrientationThreshPerMass = TypedDict('StartPickAndPlaceThreadParametersDestFilterByTargetOrientationThreshPerMass', {
    'mass': float,
    'destFilterByTargetOrientationThresh': float,
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

StartPickAndPlaceThreadParametersDropInDestInfoPerContainer = dict[str, dict[str, Any]]

StartPickAndPlaceThreadParametersExecuteITLOnCompleteLayerInfo = TypedDict('StartPickAndPlaceThreadParametersExecuteITLOnCompleteLayerInfo', {
    'cornerThresh': float,
    'heightThresh': float,
    'itlProgramName': str,
    'itlCacheMode': int,
    'numTimesToExecute': int,
    'use': bool,
}, total=False)

GrabbedTargetIOSensorDefinition = TypedDict('GrabbedTargetIOSensorDefinition', {
    'id': str,
    'deleted': bool,
    'index': int,
    'geomNames': list[str],
    'ikparamNames': list[str],
    'ioname': str,
    'rayCheckMode': Literal['all', 'any'],
}, total=False)

GrabbedTargetValidationSignalsInfo = TypedDict('GrabbedTargetValidationSignalsInfo', {
    'ioSensors': list[GrabbedTargetIOSensorDefinition],
    'use': bool,
}, total=False)

GraspDepartAccelDecelScaleMultOnTargetMassArrayElement = TypedDict('GraspDepartAccelDecelScaleMultOnTargetMassArrayElement', {
    'id': str,
    'deleted': bool,
    'index': int,
    'scale': float,
    'mass': float,
}, total=False)

GraspDepartAccelDecelScaleMultOnTargetMass = list[dict[str, Any]]

GraspGoalPairCostMultipliers = TypedDict('GraspGoalPairCostMultipliers', {
    'grabToolRotMult': float,
    'singularityPriorityMult': float,
    'sourceDestTargetOrientationDiffMult': float,
    'targetDeviationMult': float,
    'toolDeviationMult': float,
    'expectedTimeCostMult': float,
    'graspPriorityMult': float,
    'simultaneousReleaseAdditionalTargetMult': float,
    'isCollidedWithMoveLocationMult': float,
}, total=False)

GraspPriorityMultipliers = TypedDict('GraspPriorityMultipliers', {
    'constraintToolDeviationMult': float,
    'graspDistMult': float,
    'graspPriorityMult': float,
    'multiGrasp': float,
    'overlappingNonPickablePriorityMult': float,
    'suctionForceMult': float,
    'toolWorldDir': tuple[float, float, float],
    'toolWorldDirMult': float,
    'transferSpeedMult': float,
    'uncertainMaskIntersectionMult': float,
}, total=False)

IkSolverParameters = TypedDict('IkSolverParameters', {
    'ikSolverFreeIncRev': float,
    'ikSolverFreeIncPrismatic': float,
    'ikSolverJacobianRefinementErrorThresh': float,
    'ikSolverJacobianRefinementMaxIterations': int,
    'ikSolverWorkspaceDiscretizedRotationAngle': float,
    'ikSolverRotationProjectionAngleTolerance': float,
}, total=False)

InspectionFailDropOffInfo = TypedDict('InspectionFailDropOffInfo', {
    'destcoordtype': AllDestCoordType,
    'destname': str,
    'destDirectionAngle': float,
    'departOffsetAfterTorqueLimits': tuple[float, float, float],
    'dropApproachOffset': tuple[float, float, float],
    'dropDepartOffset': tuple[float, float, float],
    'dropOffLocationName': str,
    'facePlaceEdge': tuple[float, float, float],
    'isDroppedOffIOName': str,
    'stopIOName': str,
    'atEndIOCheck': bool,
    'use': bool,
    'useDropOffAfterTorqueLimits': bool,
}, total=False)

InitialDetectionValidationInfo = TypedDict('InitialDetectionValidationInfo', {
    'checkNumTargetsMatchesOrder': bool,
    'checkExpectedBarcodes': list[str],
    'timeout': float,
    'use': bool,
}, total=False)

StartPickAndPlaceThreadParametersInputPlacedPartInfoOnArrivals = TypedDict('StartPickAndPlaceThreadParametersInputPlacedPartInfoOnArrivals', {
    'poseInDest': float,
    'partType': str,
    'partUri': str,
}, total=False)

IoSignalsInfo = TypedDict('IoSignalsInfo', {
    'numLeftInOrderIOName': str,
    'numPutInDestinationIONames': list[str],
    'numPutInDestinationToAdd': int,
    'orderNumberToAdd': int,
    'detectedSizeXYZ': int,
    'detectedSizeXYZIOName': str,
    'grabbedTargetInfoIOName': str,
    'placedTargetInfoIOName': str,
}, total=False)

ITLExecutionInfo = TypedDict('ITLExecutionInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'delayNeedPlaceContainerAfterITL': bool,
    'finishAtStartAfterITL': bool,
    'finishPlanMode': Literal['startOfITL', 'startRobotConfiguration'],
    'eventType': Literal['afterOrderFinish', 'beforeOrderStart', 'postFinal', 'waitingForEnvironmentUpdate'],
    'finishCodes': list[str],
    'needPlacedTarget': bool,
    'needPickContainer': bool,
    'needPlaceContainer': bool,
    'itlProgramNames': list[str],
}, total=False)

IONameValueDefinition = TypedDict('IONameValueDefinition', {
    'ioName': str,
    'ioValue': int,
}, total=False)

CyclePreconditionIOInfo = TypedDict('CyclePreconditionIOInfo', {
    'ioNameValueList': list[IONameValueDefinition],
}, total=False)

JittererParameters = TypedDict('JittererParameters', {
    'maxJitter': float,
    'maxJitterIterations': int,
    'maxJitterLinkDist': float,
    'jitterBiasDirection': tuple[float, float, float],
    'jitterNeighDistThresh': float,
    'useWorkspaceJitterer': bool,
}, total=False)

JustInTimeToolChangePlanningParameters = TypedDict('JustInTimeToolChangePlanningParameters', {
    'forceUseFirstToolOnStart': bool,
    'programName': str,
    'toolNames': list[str],
    'use': bool,
}, total=False)

LabelPlacingInfo = TypedDict('LabelPlacingInfo', {
    'use': bool,
    'printTriggerIO': str,
    'isAtPrinterLocationIO': str,
    'labelForPartIO': str,
    'labelEjectionIkParamName': str,
    'approachOffsetDir': tuple[float, float, float],
    'departOffsetDir': tuple[float, float, float],
    'pauseTimeAtPrinterLocation': float,
    'labelSizeXY': tuple[float, float],
    'labelPositionGeneration': Literal['bottomFlat'],
}, total=False)

MinViableRegionPlanParameters = TypedDict('MinViableRegionPlanParameters', {
    'alwaysRegraspOnPartialGrasp': bool,
    'alwaysRegraspOnSizeMeasurement': bool,
    'allowLiftOnDragFailure': bool,
    'allowLiftSizeRatio': float,
    'canDoReplanWithNewTarget': bool,
    'captureTriggerIOName': str,
    'captureTriggerOutputDelay': float,
    'captureCancelIOName': str,
    'captureFinishIOName': str,
    'defaultLiftingUpSpeedMult': float,
    'doAutoRecoveryOnPieceLostOnReplan': bool,
    'draggingForceTorqueExceedStop': bool,
    'draggingForceTorqueExceedThresholds': list[float],
    'draggingForceTorqueExceedThresholdsDelta': list[float],
    'isPerpendicularDragEnabled': bool,
    'failOnMinCandidateMass': bool,
    'maxAllowedObjectComRatioForRecovery': float,
    'maxPossibleSizePaddingForOverlapCheck': float,
    'maxPossibleSizeVisibiltyPadding': float,
    'registrationDragOffset': float,
    'registrationDragLiftOffset': float,
    'registrationGraspLiftOffset': float,
    'registrationMotionMode': Literal['lift', 'drag', 'dragAndLeave', 'liftAndShift'],
    'liftingUpSpeedMultForSizeMeasurement': float,
    'minAllowedObjectSizeRatio': float,
    'maxAllowedObjectComRatio': float,
    'minAllowedTransferSpeedMult': float,
    'minCandidateMass': float,
    'minCornerVisibleDist': float,
    'minCornerVisibleDistForOctagonal': float,
    'minTargetSizeAlongWithDraggingDirectionForRegrasping': float,
    'maxSecondsToHold': float,
    'maxSaggingAngle': float,
    'numMaxRetryFromGraspingModelGenerationFailures': int,
    'partialGraspCoveringRatioThreshold': float,
    'obstacleCleanSizeXYZForReplan': list[float],
    'occlusionPenetration': float,
    'overlappingObstacleLoweringDistanceForReplan': float,
    'saggingAngleGuessingRatio': float,
    'shiftByGraspDepartOffset': bool,
    'skipDecelOnKnownHeight': bool,
    'stopSensorIkParamName': str,
    'stopSensorObjectName': str,
    'tiltAngleThresholdToAddObjectSet': float,
    'waitRegistrationForNextPlan': bool,
    'use': bool,
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

MultiPickInfo = TypedDict('MultiPickInfo', {
    'allowMultiPickForSurrounded': bool,
    'axisAlignThresh': float,
    'mode': Literal['None', 'Any', 'OnlyTwoPick', 'UpToTwoPick', 'OnlyThreePick', 'UpToThreePick', 'OnlyFourPick', 'UpToFourPick', 'OnlyFivePick', 'UpToFivePick', 'OnlySixPick', 'UpToSixPick', 'AllMultiPicks'],
    'modeOnFail': Literal['None', 'Any', 'OnlyTwoPick', 'UpToTwoPick', 'OnlyThreePick', 'UpToThreePick', 'OnlyFourPick', 'UpToFourPick', 'OnlyFivePick', 'UpToFivePick', 'OnlySixPick', 'UpToSixPick', 'AllMultiPicks'],
    'releaseMode': Literal['OneByOne', 'SimultaneousRelease', 'Any'],
    'releaseModeOnFail': Literal['OneByOne', 'SimultaneousRelease', 'Any'],
    'horzDiscretization': float,
    'placeOffset': tuple[float, float, float],
    'placeOffsetDifferentHeight': tuple[float, float, float],
    'priorityForAxisAlignMult': float,
    'priorityForPosAlignMult': float,
    'priorityForSimultaneousRelease': float,
    'priorityForTouching': float,
    'priorityPenaltyMultForNewConnectedComponent': float,
    'sameSizeThresholdForTextureless': float,
    'simultaneousReleaseMaxAllowedAngle': float,
    'simultaneousReleaseMaxAllowedBoundedWidth': float,
    'simultaneousReleaseMaxAllowedDistance': float,
    'simultaneousReleaseFrontAndBackTolerance': float,
    'surfaceThresh': float,
    'touchingDistThresh': float,
    'transferSpeedDiff': float,
    'travelDir': tuple[float, float, float],
    'use': bool,
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

ObjectTypeDynamicParametersCubicalCase = TypedDict('ObjectTypeDynamicParametersCubicalCase', {
    'transferSpeedMult': float,
    'transferSpeedPostMult': float,
    'waitTimeAfterChuckSuccess': float,
    'objectPackingId': int,
}, total=False)

ObjectTypeDynamicParametersOctagonalCase = TypedDict('ObjectTypeDynamicParametersOctagonalCase', {
    'transferSpeedMult': float,
    'transferSpeedPostMult': float,
    'waitTimeAfterChuckSuccess': float,
    'objectPackingId': int,
}, total=False)

ObjectTypeDynamicParameters = TypedDict('ObjectTypeDynamicParameters', {
    'cubicalCase': ObjectTypeDynamicParametersCubicalCase,
    'octagonalCase': ObjectTypeDynamicParametersOctagonalCase,
}, total=False)

TargetOverlapConstraintInfo = TypedDict('TargetOverlapConstraintInfo', {
    'aboveBoxOverlap': float,
    'allowPickupOverlappedTargetAfterOverlapIsGone': bool,
    'belowBoxOverlap': float,
    'belowBoxOverlapOnFail': float,
    'canDropInSourceOnFail': bool,
    'canPickupAfterOverlapIsGone': bool,
    'checkOverlapWithPreviouslyPickedTargetsOnly': bool,
    'constraintAngle': float,
    'graspDepartOverlapOffsetDir': tuple[float, float, float],
    'ignoreOverlapPointsFromWall': float,
    'ignoreOverlapPointsFromNearbyTargets': float,
    'ignoreSafetyVolumeOverlap': bool,
    'maxPickupNumOverlappingPoints': int,
    'maxPickupNumOverlappingPointsOnFail': int,
    'maxPointsOverlappedAreaRatio': float,
    'maxChangeDirNumOverlappingPoints': int,
    'neighOverlapThresh': float,
    'neighOverlapThreshForMVR': float,
    'neighOverlapThreshOnFail': float,
    'neighOverlapChangeDepartThresh': float,
    'overlapConstraintAxis': tuple[float, float, float],
    'overlapUpAxis': tuple[float, float, float],
    'overlapSafetyPadding': float,
    'overlapSafetyPaddingOnFail': float,
    'pointsOverlapAreaThresh': float,
    'pointsOverlapAreaThreshOnFail': float,
    'pointSizeIncreaseMultForOverlapChecking': float,
    'transferSpeedMultAtMaxNeighOverlapThresh': float,
    'transferSpeedMultAtMaxNeighOverlapThreshOnFail': float,
    'useOverlapToComputeGraspDepart': bool,
    'useShadowExtents': bool,
    'waitForNewDetectionAfterOnFail': bool,
    'use': bool,
}, total=False)

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

PickLocationInfo = TypedDict('PickLocationInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'containerId': str,
    'containerType': Optional[str],
    'isContainerEmptyOnChange': bool,
    'isContainerChanged': bool,
    'locationName': str,
    'ikParamName': str,
}, total=False)

PieceInspectionInfo = TypedDict('PieceInspectionInfo', {
    'ioName': str,
    'expectedIOValue': float,
    'ioCheckStartDelay': float,
    'use': bool,
}, total=False)

PlaceLocationInfo = TypedDict('PlaceLocationInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'containerId': str,
    'containerType': Optional[str],
    'isContainerEmptyOnChange': bool,
    'isContainerChanged': bool,
    'locationName': str,
    'ikParamName': str,
}, total=False)

PredictedPlaceInfo = TypedDict('PredictedPlaceInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'ikParamName': str,
    'sourcecontainername': str,
}, total=False)

PredictDetectionInfo = TypedDict('PredictDetectionInfo', {
    'alignLongAxisToX': bool,
    'alignLongAxisToY': bool,
    'boxLocalOffset': tuple[float, float, float],
    'allowPredictedMovementBeforeDetection': bool,
    'placeInfos': list[PredictedPlaceInfo],
    'sizeThreshXYZ': tuple[float, float, float],
    'use': bool,
    'useContainerRegionForPadding': bool,
    'useExistingTargetOnly': bool,
}, total=False)

PostCycleExecutionConfigurations = TypedDict('PostCycleExecutionConfigurations', {
    'circularMeasuringDistanceSpeed': float,
    'compartmentLength': float,
    'conveyorLength': float,
    'conveyorSpeed': float,
    'distSensorMaxDist': float,
    'distSensorMinDist': float,
    'distSensorName': str,
    'distSensorToPlaceOffset': float,
    'distSensorUri': str,
    'encoderRatio': float,
    'encoderReadDelay': float,
    'fLineSensorOffset': float,
    'grabbingaccelmult': float,
    'ignorePlacementBodyNames': list[str],
    'maxPlaceZDeceleration': float,
    'measuringDistancsRadius': float,
    'minBoxCheckThreshold': float,
    'minTargetHeight': float,
    'minTargetHeightForSensor': float,
    'placename': str,
    'postCycleType': Literal['', 'conveyorplace'],
    'robotOffset': float,
    'useDistSensor': bool,
    'xacceloffset': float,
    'xoffset': float,
    'zoffset': float,
}, total=False)

PutBackParameters = TypedDict('PutBackParameters', {
    'putBackSafetyForceTorqueExceedStop': bool,
    'putBackSafetyForceTorqueExceedThresholds': list[float],
    'putBackSafetyForceTorqueExceedThresholdsDelta': list[float],
    'putBackDepartOffset': float,
    'putBackSpeedMult': float,
    'putBackDistThreshold': float,
    'use': bool,
}, total=False)

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

RestrictLabelOrientationInfo = TypedDict('RestrictLabelOrientationInfo', {
    'use': bool,
    'mode': Literal['specifyAllowedOrientations', 'allowOnlyFacingOutside'],
    'coordinates': Literal['global', 'container'],
    'allowedOrientations': list[tuple[float, float, float]],
}, total=False)

SkipCollidingDestsInfo = TypedDict('SkipCollidingDestsInfo', {
    'skipEnvironmentCollidingDests': bool,
    'skipPlacedCollidingDests': bool,
    'skipUriFilteredDests': bool,
}, total=False)

BinpickingDest = TypedDict('BinpickingDest', {
    'id': str,
    'deleted': bool,
    'index': int,
    'externalCollisionNames': list[str],
    'jointvalues': list[float],
    'ikparamName': str,
    'isDropInSource': bool,
    'isDropOff': bool,
    'positionName': str,
    'validGraspSetName': str,
    'name': str,
    'validContainerUri': str,
    'validTargetUri': str,
    'use': bool,
}, total=False)

BinpickingDestGoal = TypedDict('BinpickingDestGoal', {
    'id': str,
    'deleted': bool,
    'index': int,
    'dests': list[BinpickingDest],
    'validGraspSetName': str,
}, total=False)

DestGoals = list[dict[str, Any]]

LowPriorityTypeInfo = TypedDict('LowPriorityTypeInfo', {
    'id': str,
    'deleted': bool,
    'index': int,
    'validTargetUris': list[str],
    'validObjectTypes': list[str],
    'cycleOverwrite': CycleOverwriteParameters,
    'destGoals': DestGoals,
    'use': bool,
}, total=False)

SplitterSheetInfoSchema = TypedDict('SplitterSheetInfoSchema', {
    'airchannelUsageRatio': float,
    'containerBottomThreshold': float,
    'splitterSheetSafetyVolumeSaggingFactor': float,
    'validateBySizeAndMass': bool,
    'lowPriorityTypeInfo': LowPriorityTypeInfo,
    'minSplitterSheetSize': tuple[float, float, float],
    'maxSplitterSheetMass': float,
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

StartPickAndPlaceThreadParametersSourceDynamicGoalsGeneratorParametersOverwriteUserPackFormationParameters = dict[str, Any]

StartPickAndPlaceThreadParametersSourceDynamicGoalsGeneratorParametersOverwrite = TypedDict('StartPickAndPlaceThreadParametersSourceDynamicGoalsGeneratorParametersOverwrite', {
    'allowFallbackToRandom': bool,
    'autoComputePackFormation': bool,
    'autoRotatePackFormation': bool,
    'randomPackingParameters': RandomPackingParameters,
    'skipPackFormationValidation': bool,
    'saveDynamicGoalGeneratorState': bool,
    'saveDynamicGoalGeneratorStateFailed': bool,
    'saveDynamicGoalGeneratorStateOnRandomMode': bool,
    'useComputePackFormationFromState': bool,
    'userPackFormationParameters': StartPickAndPlaceThreadParametersSourceDynamicGoalsGeneratorParametersOverwriteUserPackFormationParameters,
}, total=False)

StrictPickOrdering = TypedDict('StrictPickOrdering', {
    'axis': Literal['yp', 'zp'],
    'containerLocalAxis': Literal['xp', 'xn', 'yp', 'yn', 'zp', 'zn'],
    'criteria': Literal['zp', 'xyz', 'boxSize_xy'],
    'lowPriorityTypeInfos': list[LowPriorityTypeInfo],
    'name': Literal['ascending', 'descending'],
    'threshForAxis': float,
    'type': Literal['chooseOneType', 'lowPriorityTypes'],
    'use': bool,
    'XMult': float,
    'YMult': float,
    'ZMult': float,
}, total=False)

TargetPriorityMultipliers = TypedDict('TargetPriorityMultipliers', {
    'failureCountdownMult': float,
    'dragDirectionBlockedMult': float,
    'extraOffsetMult': float,
    'mvrCornerOcclusionPresentMult': float,
    'positiveXYZMult': tuple[float, float, float],
    'negativeXYZMult': tuple[float, float, float],
    'discretizationXYZ': float,
    'angleMult': float,
    'centerDistXYMult': float,
    'centerDistXYMinThresh': float,
    'targetConfidenceMult': float,
    'nonOverlappedMult': float,
    'xyAreaMult': float,
    'occludedEdgeMult': float,
    'uncertainCornerMult': float,
    'prevPickedTargetsDistMult': float,
    'latestPickedTargetXYZDistMult': tuple[float, float, float],
}, total=False)

TargetRotationConstraintParameters = TypedDict('TargetRotationConstraintParameters', {
    'use': bool,
    'permissibleTargetZRotationRangeMin': float,
    'permissibleTargetZRotationRangeMax': float,
    'permissibleSquareTargetZRotationRangeMin': float,
    'permissibleSquareTargetZRotationRangeMax': float,
    'toolMaxRotationBetweenPickAndPlace': float,
}, total=False)

TargetLabelAlignmentTargetsOnSourceContainerEdgeParameters = TypedDict('TargetLabelAlignmentTargetsOnSourceContainerEdgeParameters', {
    'use': bool,
    'distanceBufferOnSourceContainerEdge': float,
    'labelDirectionsInSourceContainer': list[Literal['None', 'PX', 'NX', 'PY', 'NY']],
    'labelOnSquareTargetDirectionsInSourceContainer': list[Literal['None', 'PX', 'NX', 'PY', 'NY']],
    'isLabelAlwaysOnShorterEdge': bool,
    'isLabelAlwaysOnOuterFaces': bool,
}, total=False)

TargetLabelAlignmentParameters = TypedDict('TargetLabelAlignmentParameters', {
    'use': bool,
    'labelDirectionAtDestination': Literal['None', 'PX', 'NX', 'PY', 'NY'],
    'coordinateFrameOfLabelDirectionAtDestination': Literal['destIkParameter', 'destContainer', 'destLocation', 'world'],
    'labelDirectionsInSourceContainer': list[Literal['None', 'PX', 'NX', 'PY', 'NY']],
    'labelOnSquareTargetDirectionsInSourceContainer': list[Literal['None', 'PX', 'NX', 'PY', 'NY']],
    'isLabelAlwaysOnShorterEdge': bool,
    'isLabelAlwaysOnOuterFaces': bool,
    'targetsOnSourceContainerEdgeParameters': TargetLabelAlignmentTargetsOnSourceContainerEdgeParameters,
}, total=False)

StartPickAndPlaceThreadParametersToolposesCycleStart = TypedDict('StartPickAndPlaceThreadParametersToolposesCycleStart', {
    'ikparamname': str,
    'toolname': str,
    'useSourceContainerIkParams': bool,
}, total=False)

StartPickAndPlaceThreadParametersToolposes = TypedDict('StartPickAndPlaceThreadParametersToolposes', {
    'cycleStart': StartPickAndPlaceThreadParametersToolposesCycleStart,
}, total=False)

ToolSpeedAccelInfo = TypedDict('ToolSpeedAccelInfo', {
    'use': bool,
    'maxGrabbingManipAccel': float,
    'maxGrabbingManipSpeed': float,
    'maxFreeManipAccel': float,
    'maxFreeManipSpeed': float,
}, total=False)

TransferSpeedMultPerWeight = TypedDict('TransferSpeedMultPerWeight', {
    'id': str,
    'deleted': bool,
    'index': int,
    'transferSpeedPostMult': float,
    'weight': float,
}, total=False)

TransferTrajectoryCostMultipliers = TypedDict('TransferTrajectoryCostMultipliers', {
    'shelfPickingTransferCostMult': float,
}, total=False)

YKKControlInfo = TypedDict('YKKControlInfo', {
    'deviceCompartmentSize': float,
    'devicePositionResetIO': list[str],
    'deviceMoveIO': list[str],
    'distSensorPartClampedJigThreshold': float,
    'distSensorUri': str,
    'initialPosition': int,
    'jigCloseIO': list[str],
    'jigOpenIO': list[str],
    'numDevicePositions': int,
    'timeToMoveOneCompartment': float,
    'timeToCloseJig': float,
    'timeToOpenJig': float,
    'useDistSensor': bool,
}, total=False)

StartPickAndPlaceThreadParametersControllerclientparametersRobotBridgeConnectionInfo = TypedDict('StartPickAndPlaceThreadParametersControllerclientparametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

StartPickAndPlaceThreadParametersControllerclientparameters = TypedDict('StartPickAndPlaceThreadParametersControllerclientparameters', {
    'controllerpassword': str,
    'controllerurl': str,
    'controllerusername': str,
    'robotBridgeConnectionInfo': StartPickAndPlaceThreadParametersControllerclientparametersRobotBridgeConnectionInfo,
    'scenepk': str,
    'slaverequestid': str,
    'taskheartbeatport': int,
    'taskheartbeattimeout': int,
    'tasktype': str,
    'taskzmqport': int,
}, total=False)

StartPickAndPlaceThreadParametersDestSensorSelectionInfosArrayElement = TypedDict('StartPickAndPlaceThreadParametersDestSensorSelectionInfosArrayElement', {
    'sensorName': str,
    'sensorLinkName': str,
}, total=False)

StartPickAndPlaceThreadParametersDetectionInfosArrayElement = TypedDict('StartPickAndPlaceThreadParametersDetectionInfosArrayElement', {
    'containerDetectionMode': str,
    'locationName': str,
}, total=False)

StartPickAndPlaceThreadParametersItlParameters = dict[str, Any]

StartPickAndPlaceThreadParametersLocationCollisionInfosArrayElement = TypedDict('StartPickAndPlaceThreadParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

StartPickAndPlaceThreadParametersRegistrationInfo = TypedDict('StartPickAndPlaceThreadParametersRegistrationInfo', {
    'controllerusername': str,
    'registrationIp': str,
    'registrationPort': int,
    'registrationpassword': str,
    'registrationurl': str,
    'registrationusername': str,
}, total=False)

StartPickAndPlaceThreadParametersSourceSensorSelectionInfosArrayElement = TypedDict('StartPickAndPlaceThreadParametersSourceSensorSelectionInfosArrayElement', {
    'sensorName': str,
    'sensorLinkName': str,
}, total=False)

StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopPickPlaceThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StopPickPlaceThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopPickPlaceThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopPickPlaceThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPickPlaceStatusParametersDynamicEnvironmentStateMapValue = TypedDict('GetPickPlaceStatusParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPickPlaceStatusParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPickPlaceStatusParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetPickPlaceStatusReturns = TypedDict('GetPickPlaceStatusReturns', {
    'status': int,
    'error': str,
}, total=False)

ComputeIKParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeIKParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeIKParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeIKParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeIKParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeIKParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeIKParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeIKParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeIKParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeIKParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeIKParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ComputeIKReturns = TypedDict('ComputeIKReturns', {
    'solutions': list[Any],
    'errors': list[str],
}, total=False)

InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValue = TypedDict('InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': InitializePartsWithPhysicsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

InitializePartsWithPhysicsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StopPhysicsThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopPhysicsThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopPhysicsThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopPhysicsThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopPhysicsThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopPhysicsThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StopPhysicsThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopPhysicsThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopPhysicsThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopPhysicsThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopPhysicsThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValue = TypedDict('JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': JitterPartUntilValidGraspParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

JitterPartUntilValidGraspParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveToDropOffParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveToDropOffParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveToDropOffParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveToDropOffParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveToDropOffParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveToDropOffParametersDynamicEnvironmentStateMapValue = TypedDict('MoveToDropOffParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveToDropOffParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveToDropOffParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveToDropOffParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveToDropOffParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveToDropOffParametersLocationCollisionInfosArrayElement = TypedDict('MoveToDropOffParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

MoveToDropOffParametersGripperInfoVariantItemPrefix0 = dict[str, Any]

MoveToDropOffParametersStartJointConfigurationStatesArrayElement = TypedDict('MoveToDropOffParametersStartJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveToDropOffParametersGoalJointConfigurationStatesArrayElement = TypedDict('MoveToDropOffParametersGoalJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveToDropOffParametersRobotBridgeConnectionInfo = TypedDict('MoveToDropOffParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

MoveToDropOffParametersDropOffInfo = TypedDict('MoveToDropOffParametersDropOffInfo', {
    'bottomScanPlacementInfo': BottomScanPlacementInfo,
    'constraintToolInfo': ConstraintToolInfo,
    'destDirectionAngle': float,
    'destGoals': DestGoals,
    'destcoordtype': AllDestCoordType,
    'dropApproachOffset': tuple[float, float, float],
    'dropOffLocationName': str,
    'stopIOName': str,
}, total=False)

IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValue = TypedDict('IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': IsRobotOccludingBodyParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

IsRobotOccludingBodyParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

IsRobotOccludingBodyReturns = dict[str, Any]

GetPickedPositionsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPickedPositionsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPickedPositionsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPickedPositionsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPickedPositionsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPickedPositionsParametersDynamicEnvironmentStateMapValue = TypedDict('GetPickedPositionsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPickedPositionsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPickedPositionsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPickedPositionsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPickedPositionsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValue = TypedDict('GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPickAndPlaceLogParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPickAndPlaceLogParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValue = TypedDict('MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

PausePickPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PausePickPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PausePickPlaceParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PausePickPlaceParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PausePickPlaceParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PausePickPlaceParametersDynamicEnvironmentStateMapValue = TypedDict('PausePickPlaceParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PausePickPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PausePickPlaceParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PausePickPlaceParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PausePickPlaceParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ResumePickPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ResumePickPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ResumePickPlaceParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ResumePickPlaceParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ResumePickPlaceParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ResumePickPlaceParametersDynamicEnvironmentStateMapValue = TypedDict('ResumePickPlaceParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ResumePickPlaceParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ResumePickPlaceParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ResumePickPlaceParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ResumePickPlaceParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SendStateTriggerParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SendStateTriggerParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SendStateTriggerParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SendStateTriggerParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SendStateTriggerParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SendStateTriggerParametersDynamicEnvironmentStateMapValue = TypedDict('SendStateTriggerParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SendStateTriggerParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SendStateTriggerParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SendStateTriggerParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SendStateTriggerParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetBinpickingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetBinpickingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetBinpickingStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetBinpickingStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetBinpickingStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetBinpickingStateParametersDynamicEnvironmentStateMapValue = TypedDict('GetBinpickingStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetBinpickingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetBinpickingStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetBinpickingStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetBinpickingStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValue = TypedDict('SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

PutPartsBackParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('PutPartsBackParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

PutPartsBackParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

PutPartsBackParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

PutPartsBackParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

PutPartsBackParametersDynamicEnvironmentStateMapValue = TypedDict('PutPartsBackParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[PutPartsBackParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': PutPartsBackParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': PutPartsBackParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

PutPartsBackParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValue = TypedDict('GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GenerateGraspModelFromIkParamsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GenerateGraspModelFromIkParamsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

CheckGraspModelIkParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('CheckGraspModelIkParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

CheckGraspModelIkParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

CheckGraspModelIkParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

CheckGraspModelIkParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

CheckGraspModelIkParametersDynamicEnvironmentStateMapValue = TypedDict('CheckGraspModelIkParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[CheckGraspModelIkParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': CheckGraspModelIkParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': CheckGraspModelIkParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

CheckGraspModelIkParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValue = TypedDict('SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ClearVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ClearVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ClearVisualizationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ClearVisualizationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ClearVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ClearVisualizationParametersDynamicEnvironmentStateMapValue = TypedDict('ClearVisualizationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ClearVisualizationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ClearVisualizationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ClearVisualizationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ClearVisualizationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetPlanStatisticsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPlanStatisticsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPlanStatisticsParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPlanStatisticsParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPlanStatisticsParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPlanStatisticsParametersDynamicEnvironmentStateMapValue = TypedDict('GetPlanStatisticsParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPlanStatisticsParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPlanStatisticsParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPlanStatisticsParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPlanStatisticsParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValue = TypedDict('SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValue = TypedDict('ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ManuallyPlacePackItemParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ManuallyPlacePackItemParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ManuallyPlacePackItemParametersDynamicGoalsGeneratorParametersUserPackFormationParameters = dict[str, Any]

ManuallyPlacePackItemParametersDynamicGoalsGeneratorParameters = TypedDict('ManuallyPlacePackItemParametersDynamicGoalsGeneratorParameters', {
    'allowFallbackToRandom': bool,
    'autoComputePackFormation': bool,
    'autoRotatePackFormation': bool,
    'randomPackingParameters': RandomPackingParameters,
    'skipPackFormationValidation': bool,
    'saveDynamicGoalGeneratorState': bool,
    'saveDynamicGoalGeneratorStateFailed': bool,
    'saveDynamicGoalGeneratorStateOnRandomMode': bool,
    'useComputePackFormationFromState': bool,
    'userPackFormationParameters': ManuallyPlacePackItemParametersDynamicGoalsGeneratorParametersUserPackFormationParameters,
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


