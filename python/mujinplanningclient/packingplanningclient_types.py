
# This file contains the types used in the client
from typing import TypedDict, Union, Any, Optional, Literal
from typing_extensions import Required

StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartPackFormationComputationThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StartPackFormationComputationThreadParametersDetectionInfosArrayElement = TypedDict('StartPackFormationComputationThreadParametersDetectionInfosArrayElement', {
    'containerDetectionMode': str,
    'locationName': str,
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

StartPackFormationComputationThreadParametersPackLocationInfo = TypedDict('StartPackFormationComputationThreadParametersPackLocationInfo', {
    'containerId': Optional[str],
    'containerType': Optional[str],
    'locationName': str,
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

StartPackFormationComputationThreadParametersBodyOcclusionInfosArrayElement = dict[str, Any]

StartPackFormationComputationThreadParametersLocationCollisionInfosArrayElement = TypedDict('StartPackFormationComputationThreadParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

StartPackFormationComputationThreadParametersDestSensorSelectionInfosArrayElement = TypedDict('StartPackFormationComputationThreadParametersDestSensorSelectionInfosArrayElement', {
    'sensorName': str,
    'sensorLinkName': str,
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

JittererParameters = TypedDict('JittererParameters', {
    'maxJitter': float,
    'maxJitterIterations': int,
    'maxJitterLinkDist': float,
    'jitterBiasDirection': tuple[float, float, float],
    'jitterNeighDistThresh': float,
    'useWorkspaceJitterer': bool,
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

StartPackFormationComputationThreadParametersJustInTimeToolChangePlanning = TypedDict('StartPackFormationComputationThreadParametersJustInTimeToolChangePlanning', {
    'toolNames': list[Optional[str]],
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

StartPackFormationComputationThreadParametersPackInputPartInfosArrayElement = dict[str, Any]

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

RandomPackingParameters = TypedDict('RandomPackingParameters', {
    'startSearchCorner': int,
    'intAvoidOldGoalsNeighDelta': int,
}, total=False)

StartPackFormationComputationThreadParametersDynamicGoalsGeneratorParametersUserPackFormationParameters = dict[str, Any]

StartPackFormationComputationThreadParametersDynamicGoalsGeneratorParameters = TypedDict('StartPackFormationComputationThreadParametersDynamicGoalsGeneratorParameters', {
    'allowFallbackToRandom': bool,
    'autoComputePackFormation': bool,
    'autoRotatePackFormation': bool,
    'randomPackingParameters': RandomPackingParameters,
    'skipPackFormationValidation': bool,
    'saveDynamicGoalGeneratorState': bool,
    'saveDynamicGoalGeneratorStateFailed': bool,
    'saveDynamicGoalGeneratorStateOnRandomMode': bool,
    'useComputePackFormationFromState': bool,
    'userPackFormationParameters': StartPackFormationComputationThreadParametersDynamicGoalsGeneratorParametersUserPackFormationParameters,
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

StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValue = TypedDict('StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartSingleSKUPackFormationComputationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartSingleSKUPackFormationComputationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValue = TypedDict('StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StopPackFormationComputationThreadParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StopPackFormationComputationThreadParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

StopPackFormationComputationThreadParametersRobotBridgeConnectionInfo = TypedDict('StopPackFormationComputationThreadParametersRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

VisualizePackingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('VisualizePackingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

VisualizePackingStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

VisualizePackingStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

VisualizePackingStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

VisualizePackingStateParametersDynamicEnvironmentStateMapValue = TypedDict('VisualizePackingStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[VisualizePackingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': VisualizePackingStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': VisualizePackingStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

VisualizePackingStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPackFormationSolutionParametersDynamicEnvironmentStateMapValue = TypedDict('GetPackFormationSolutionParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPackFormationSolutionParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPackFormationSolutionParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValue = TypedDict('SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': SendPackFormationComputationResultParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

SendPackFormationComputationResultParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValue = TypedDict('GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetLatestPackFormationResultListParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetLatestPackFormationResultListParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

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

StartValidatePackFormationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('StartValidatePackFormationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

StartValidatePackFormationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

StartValidatePackFormationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

StartValidatePackFormationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

StartValidatePackFormationParametersDynamicEnvironmentStateMapValue = TypedDict('StartValidatePackFormationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[StartValidatePackFormationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': StartValidatePackFormationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': StartValidatePackFormationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

StartValidatePackFormationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValue = TypedDict('ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ComputeSamePartPackResultBySimulationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

HasDetectionObstaclesParametersDynamicEnvironmentStateMapValue = TypedDict('HasDetectionObstaclesParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': HasDetectionObstaclesParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

HasDetectionObstaclesParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

HasDetectionObstaclesParametersDetectionInfosArrayElement = TypedDict('HasDetectionObstaclesParametersDetectionInfosArrayElement', {
    'containerDetectionMode': str,
    'locationName': str,
}, total=False)

HasDetectionObstaclesParametersPackLocationInfo = TypedDict('HasDetectionObstaclesParametersPackLocationInfo', {
    'containerId': Optional[str],
    'containerType': Optional[str],
    'locationName': str,
}, total=False)

HasDetectionObstaclesParametersBodyOcclusionInfosArrayElement = dict[str, Any]

HasDetectionObstaclesParametersLocationCollisionInfosArrayElement = TypedDict('HasDetectionObstaclesParametersLocationCollisionInfosArrayElement', {
    'containerName': str,
    'externalCollisionName': str,
    'forceDisableCollisionForPlanning': bool,
    'forceEnableAllLinks': bool,
    'locationName': str,
    'moveRegionLocationName': str,
    'setToLastPlaced': bool,
    'useAABB': bool,
}, total=False)

HasDetectionObstaclesParametersDestSensorSelectionInfosArrayElement = TypedDict('HasDetectionObstaclesParametersDestSensorSelectionInfosArrayElement', {
    'sensorName': str,
    'sensorLinkName': str,
}, total=False)

HasDetectionObstaclesParametersJustInTimeToolChangePlanning = TypedDict('HasDetectionObstaclesParametersJustInTimeToolChangePlanning', {
    'toolNames': list[Optional[str]],
}, total=False)

GetPackingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('GetPackingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

GetPackingStateParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

GetPackingStateParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

GetPackingStateParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

GetPackingStateParametersDynamicEnvironmentStateMapValue = TypedDict('GetPackingStateParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[GetPackingStateParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': GetPackingStateParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': GetPackingStateParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

GetPackingStateParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

SensorName = str

SensorLinkName = str

SensorSelectionInfo = TypedDict('SensorSelectionInfo', {
    'sensorName': SensorName,
    'sensorLinkName': SensorLinkName,
}, total=False)

GetPackingStateReturnsOcclusionResultsVariantItemPrefix0ArrayElement = TypedDict('GetPackingStateReturnsOcclusionResultsVariantItemPrefix0ArrayElement', {
    'sensorSelectionInfo': SensorSelectionInfo,
    'bodyname': str,
    'isocclusion': int,
    'occlusionId': int,
}, total=False)

GetPackingStateReturnsActiveLocationTrackingInfosArrayElement = TypedDict('GetPackingStateReturnsActiveLocationTrackingInfosArrayElement', {
    'locationName': str,
    'containerId': str,
    'containerName': str,
    'containerUsage': str,
    'cycleIndex': str,
    'rejectContainerIds': list[str],
    'executedOrderCycles': list[str],
    'sendMoveOutOnEachPlace': bool,
}, total=False)

GetPackingStateReturnsRobotBridgeConnectionInfo = TypedDict('GetPackingStateReturnsRobotBridgeConnectionInfo', {
    'host': str,
    'port': int,
    'queueid': str,
    'use': bool,
}, total=False)

GetPackingStateReturns = TypedDict('GetPackingStateReturns', {
    'unitInfo': UnitInfo,
    'timestamp': float,
    'occlusionResults': Optional[list[dict[str, Any]]],
    'activeLocationTrackingInfos': list[GetPackingStateReturnsActiveLocationTrackingInfosArrayElement],
    'statusMove': str,
    'statusDescMove': str,
    'realTimeVersion': str,
    'robotBridgeConnectionInfo': GetPackingStateReturnsRobotBridgeConnectionInfo,
    'isRobotBridgeExternalIOPublishing': bool,
    'isDynamicEnvironmentStateEmpty': bool,
}, total=False)

ValidatePackFormationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement = TypedDict('ValidatePackFormationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement', {
    'jointName': str,
    'connectedBodyName': str,
    'jointValue': float,
}, total=False)

ValidatePackFormationParametersDynamicEnvironmentStateMapValueLinkstatesMapValue = dict[str, Any]

ValidatePackFormationParametersDynamicEnvironmentStateMapValueLinkstates = dict[str, dict[str, Any]]

ValidatePackFormationParametersDynamicEnvironmentStateMapValueTemplateinfos = dict[str, Any]

ValidatePackFormationParametersDynamicEnvironmentStateMapValue = TypedDict('ValidatePackFormationParametersDynamicEnvironmentStateMapValue', {
    'animate': int,
    'boxFullSize': list[float],
    'cloneOriginalBodyName': str,
    'collision': bool,
    'dofvalues': list[float],
    'exclusive': bool,
    'grabbedby': tuple[str, str],
    'iscreated': bool,
    'jointConfigurationStates': list[ValidatePackFormationParametersDynamicEnvironmentStateMapValueJointConfigurationStatesArrayElement],
    'linkstates': ValidatePackFormationParametersDynamicEnvironmentStateMapValueLinkstates,
    'linkenable': str,
    'linkvisible': str,
    'pose': list[float],
    'restore': bool,
    'templateinfos': ValidatePackFormationParametersDynamicEnvironmentStateMapValueTemplateinfos,
    'uri': str,
    'visible': bool,
}, total=False)

ValidatePackFormationParametersDynamicEnvironmentState = dict[str, dict[str, Any]]

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

AABB = TypedDict('AABB', {
    'pos': tuple[float, float, float],
    'extents': tuple[float, float, float],
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


