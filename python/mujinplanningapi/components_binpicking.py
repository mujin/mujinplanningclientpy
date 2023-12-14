# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from . import AsRequired
from mujincommon.dictutil import MergeDicts

import sys
if sys.version_info[0] >= 3:
    raise ImportError("This file cannot be used until binpickingui is ported to python3, or we put the binpicking schemas in a separate module")

from . import components
# TODO(felixvd): Move into this repository
from mujinbinpickingmanager.schema import binpickingparametersschema, destGoalsSchema, distanceMeasurementInfoSchema, dynamicgoalsconfigschema, packformationparametersschema


regionname = {
    'description': _('Name of the region of the objects.'),
    'mapsTo': 'containername',
    'type': 'string',
}

unitMassSchema = {
    'type': 'string',
    'default': 'kg',
}

checkObstacleNamesSchema = {
    "type": "array",
    "items": {
        "type": "string"
    }
}

csvHeaderDeprecatedSchema = {
    '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
    'type': 'array',
    'items': {
        'type': 'string',
    },
    'deprecated': True,
}

detectionInfosSchema = {
    'type': 'array',
    'items': {
        'type': 'object',
        'properties': {
            'containerDetectionMode': {
                'type': 'string',
            },
            'locationName': {
                'type': 'string',
            },
        }
    }
}

destcontainernamesSchema = {
    'type': 'array',
    'items': {
        'type': 'string'
    }
}

destSensorSelectionInfosSchema = {
    'type': 'array',
    'items': {
        'type': 'object',
        'properties': {
            'sensorName': {
                'type': 'string',
            },
            'sensorLinkName': {
                'type': 'string',
            }
        }
    }
}

forceMoveToFinishSchema = {
    'description': _('If True, then the robot will add a finish position to the cycle even if "finish" is not present, unless "ignoreFinishPosition" is True in the order cycle command. "ignoreFinishPosition" overrides this parameter.'),
    'type': 'boolean',
}

ignoreStartPositionSchema = {
    'description': _('True if the robot should ignore going to start position'),
    'type': 'boolean',
}

packContainerTypeSchema = {
    'type': 'string',
}

packInputPartInfoSchema = {
    'type': 'object'
}

packInputPartInfosSchema = {
    'type': 'array',
    'items': packInputPartInfoSchema
}

packLocationInfoSchema = {
    'type': 'object',
    'properties': {
        'containerId': {
            'type': ['string', 'null'],
        },
        'containerType': {
            'type': ['string', 'null'],
        },
        'locationName': {
            'type': 'string',
        }
    }
}

poseSchema = {
    'type': 'array',
    'items': {
        'type': 'number',
    }
}

predictDetectionInfoSchema = MergeDicts(
    [
        binpickingparametersschema.predictDetectionInfoSchema,
        {
            'properties': {
                'isAllowPredictedMovementBeforeDetection': {
                    '$comment': 'Set in some configs but not used in the code.',
                    'type': 'boolean',
                    'deprecated': True,
                }
            }
        }
    ],
    deepcopy=True,
)[0]

# TODO(andriy.logvin): Remove after these changes are landed to schema in https://git.mujin.co.jp/dev/binpickingui/-/merge_requests/2411
pieceInspectionInfoSchema = {
    'typeName': 'PieceInspectionInfo',
    'title':_('Piece Inspection Info'),
    'description':_('Piece inspection settings at the middest'),
    'type': 'object',
    'properties': {
        'expectedIOValue': {
            'title': _('Expected Piece Inspection IO Value'),
            'description': _('The expected value of the IO specified by "ioName".'),
            'type': 'number',
            'tags':['motion', 'medium', 'si']
        },
        'ioCheckStartDelay': {
            'title': _('Piece Inspection IO Check Start Delay'),
            'description': _('seconds. Sometimes it takes time to read the piece inspection IO value, and this config compensates for the delay. The value should be smaller than midDestWaitTime.'),
            'type': 'number',
            'tags':['motion', 'advanced', 'si']
        },
        'ioName': {
            'title': _('Piece Inspection IO Name'),
            'description': _('The IO name to check its value during piece inspection. "expectedIOValue" should also be set.'),
            'type': 'string',
            'tags':['motion', 'medium', 'si']
        },
        'use':{
            'title': _('Use Piece Inspection Info'),
            'description': _('If True and the "moveToMidDest" is also True, then does piece inspection at the mid dest specified by "midDestIkparamNames" and "midDestCoordType" for "midDestWaitTime".'),
            'type': 'boolean',
            'default': False,
            'tags':['motion', 'medium', 'si']
        }
    },
    'tags':['motion', 'medium', 'si']
}

binpickingParametersSchema= MergeDicts(
    [
        binpickingparametersschema.binpickingParametersSchema,
        {
            'properties': {
                'pickContainerHasOnlyOnePart': {
                    'type': 'boolean',
                },
                'finalPlanRobotConfiguration': {
                    'type': 'string',
                },
                'destGoals': destGoalsSchema.destGoalsSchema, # Was migrated out of binpickingParametersSchema to containerProperties, but is still a valid property of this function.
                'registrationInfo': {
                    'type': 'object',
                    'properties': {
                        'controllerusername': {
                            'type': 'string',
                        },
                        'registrationIp': {
                            'type': 'string',
                        },
                        'registrationPort': {
                            'type': 'integer',
                        },
                        'registrationpassword': {
                            'type': 'string',
                        },
                        'registrationurl': {
                            'type': 'string',
                        },
                        'registrationusername': {
                            'type': 'string',
                        },
                    },
                },
                'destcontainernames': destcontainernamesSchema,  # In 20220127_binpicking.py migrated to destContainerInfo
                'worksteplength': {  # In 20220412_planningcommon_initial.py was deleted
                    'type': 'number',
                },
                'deletetarget': {
                    'description': _('Whether to delete target after pick and place is done.'),
                    'type': 'boolean',
                },
                'passOnDropAtDestinationNames': {  # Comment from Rosen 5 years ago: "passOnDropAtDestinationNames is deprecated, should be in containerProperties"
                    'type': 'array',
                    'items': {
                        'type': 'string',
                    }
                },
                'pieceInspectionInfo': pieceInspectionInfoSchema,
                'forceMoveToFinish': forceMoveToFinishSchema,  # In conf but not in binpiskingparameters
                'forceStartRobotPositionConfigurationName': {  # Accepted by the server but not in conf.
                    'description': _('If not None, then have the robot start with this position configuration regardless of what is in orderIds or robot positions/connected body active states.'),
                    'type': ['string', 'null'],
                },
                'initiallyDisableRobotBridge': {
                    'description': _('If True, stops any communication with the robotbridge until robot bridge is enabled.'),
                    'type': 'boolean',
                },
                'departoffsetdir': components.departoffsetdir,  # Moved to graspDepartOffsetDir in 20220412_planningcommon_initial.py
                'itlParameters': {  # In conf but not in binpiskingparameters
                    'type': 'object',
                    'additionalProperties': True,
                },
                'controllerclientparameters': {
                    'type': 'object',
                    'properties': {
                        'controllerpassword': {
                            'type': 'string',
                        },
                        'controllerurl': {
                            'type': 'string',
                        },
                        'controllerusername': {
                            'type': 'string',
                        },
                        'robotBridgeConnectionInfo': components.robotBridgeConnectionInfo,
                        'scenepk': {
                            'type': 'string',
                        },
                        'slaverequestid': {
                            'type': 'string',
                        },
                        'taskheartbeatport': {
                            'type': 'integer',
                        },
                        'taskheartbeattimeout': {
                            'type': 'integer',
                        },
                        'tasktype': {
                            'type': 'string',
                        },
                        'taskzmqport': {
                            'type': 'integer',
                        },
                    },
                },
                'sourceSensorSelectionInfos': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'properties': {
                            'sensorName': {
                                'type': 'string',
                            },
                            'sensorLinkName': {
                                'type': 'string',
                            }
                        }
                    }
                },
                # 'checkObstacleNames',
                'detectionInfos': detectionInfosSchema,
                #'destikparamnames',
                'toolname': {  # In conf but not in binpiskingparameters
                    'type': 'string',
                },
                'cycleIndex': {
                    'type': 'string',
                },
                'containername': {
                    'type': ['string', 'null'],
                },
                'placedTargetPrefixes': {
                    'type': 'array',
                    'items': {
                        'type': 'string',
                    }
                },
                # 'toolposes',
                # 'pickFailureDepartRetryWidth',
                'deleteTargetDestInfo': {  # removed in goalparams.py
                    'type': 'object',
                    'properties': {
                        'use': {
                            'type': 'boolean',
                        }
                    }
                },
                'ignoreStartPosition': ignoreStartPositionSchema,  # In conf but not in binpiskingparameters
                'locationCollisionInfos': components.locationCollisionInfos,
                'sourcecontainernames': {  # In conf but not in binpiskingparameters
                    'type': 'array',
                    'items': {
                        'type': 'string',
                    }
                },
                'cycleStartUseToolPose': {  # In conf but not in binpiskingparameters
                    'description': _('True if the robot should go to the tool position rather than joint values at the start of the cycle'),
                    'type': 'boolean',
                },
                'destSensorSelectionInfos': destSensorSelectionInfosSchema,
                'destdepartoffsetdir': components.departoffsetdir,  # Migrated in 20230505_approachDepartOffsets.py
                'deleteTargetWhenPlacedInDest': {
                    'type': ['string', 'boolean'],
                    'enum': ['DeleteInAll', 'KeepInAll', True, False],
                },
                'randomBoxInfo': {
                    'properties': {
                        'objectWeight': {
                            '$comment': 'Deprecated 2022/01/08, use objectMass.',
                            'title': 'Object weight',
                            'description': _('kg, specifies weight of random box. Same as objectMass.'),
                            'deprecated': True,
                            'type': 'number',
                            'minimum': 0.01,
                            'default': 1.0,
                            'tags':['motion', 'advanced', 'dev', 'target']
                        },
                        'generateCornerOffsets': {
                            '$comment': 'Present in some configs but not used in the code.',
                            'type': 'boolean',
                            'deprecated': True,
                        },
                    }
                },
                'dynamicGoalsGeneratorParameters': {
                    'properties': {
                        'moduleConfigurationParameters': {
                            'properties': {
                                'useLayoutData': {
                                    'type': 'boolean',
                                    'enum': [0, 1, False, True],
                                }
                            }
                        },
                        'placementConstraintParameters': {
                            'properties': {
                                'intJitterPlacementOffset': {
                                    'type': 'integer',
                                }
                            }
                        },
                        # TODO(andriy.logvin): Move to schema after https://git.mujin.co.jp/dev/packingcommon/-/merge_requests/10 is merged.
                        'enableWallSwitchingApproach': {
                            'type': 'boolean',
                        },
                        # TODO(andriy.logvin): Remove after configs are migrated. Was deprecated in https://git.mujin.co.jp/dev/binpickingui/-/commit/125686b0531590c212fc97198c2936c13f5e72e5 .
                        'usePlacementPriorities': {
                            'deprecated': True,
                            'type': 'boolean',
                        },
                    }
                },
                'predictDetectionInfo': predictDetectionInfoSchema,
                'savetrajectorylog': {
                    'type': 'boolean',
                    'enum': [0, 1, False, True],
                },
                'sourceDynamicGoalsGeneratorParametersOverwrite': {
                    'properties': {
                        'moduleConfigurationParameters': {
                            'properties': {
                                'useLayoutData': {
                                    'type': 'boolean',
                                    'enum': [0, 1, False, True],
                                }
                            }
                        }
                    }
                },
                'waitForStateTrigger': {
                    'type': ['string', 'null'],
                }
            },
        },
    ],
    deepcopy=True,
)[0]

packFormationResultSchema = {
    'type': 'object',
    'properties': {
        'containerInnerFullSize': {
            'type': 'array',
            'items': {
                'type': 'number'
            }
        },
        'comPackedItems': components.Vector3Schema,
        'containerName': {
            'type': 'string',
        },
        'destContainerName': {
            'type': 'string',
        },
        'innerEmptyRegionPose': poseSchema,
        'packContainerType': packContainerTypeSchema,
        'packageDimensions': components.Vector3Schema,
        'vContainerInnerExtents': {
            'type': 'array',
            'items': {
                'type': 'number',
            }
        },
        'vPackingItems': {
            'type': 'array',
            'items': {
                'type': 'object',
            },
        },
        'useAutoPackFormationComputation': {
            'type': 'boolean',
        },
        'useComputePackFormationFromState': {
            'type': 'boolean',
        },
        'unit': components.unit,
        'clientID': {
            '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
            'type': 'string',
            'deprecated': True,
        },
        'csvHeader': csvHeaderDeprecatedSchema,
        'unitMass': MergeDicts(
            [
                unitMassSchema,
                {
                    '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
                    'deprecated': True,
                }
            ],
            deepcopy=True,
        )[0],
    }
}

validatedPackFormationResultListSchema = {
    'type': 'array',
    'items': {
        'type': 'object',
        'properties': MergeDicts(
            [
                OrderedDict([
                    ('validationStatus', {
                        'type': 'string',
                    }),
                    ('validationErrorCode', {
                        'type': 'integer',
                    }),
                    ('validationErrorDesc', {
                        'type': 'string'
                    }),
                    ('fillRatio', {
                        'type': 'number',
                    }),
                    ('packContainerPose', poseSchema),
                ]),
                packFormationResultSchema['properties'],
            ],
            deepcopy=True,
        )[0]
    },
}

validatePackFormationResultListParametersSchema = {
    'type': 'object',
    'properties': OrderedDict([
        ('packFormationResultList', {
            'type': 'array',
            'isRequired': True,
            'items': packFormationResultSchema,
        }),
        ('robotname', AsRequired(components.robotname)),
        ('toolname', AsRequired(components.toolname)),
        ('packLocationInfo', packLocationInfoSchema),
        ('doPlacementValidation', {
            'description': 'If True will do placmeent validation by calling FindNextFreePlacementGoals for each placed item.',
            'type': 'boolean',
        }),
        ('forceValidatePackContainerType', {
            'type': 'boolean',
        }),
        ('checkObstacleNames', checkObstacleNamesSchema),
        ('unit', components.unit),
        ('destcontainernames', destcontainernamesSchema),
        ('containername', {
            'type': 'string',
        }),
        ('destcontainername', {
            'type': 'string',
        }),
        ('locationName', {
            'type': 'string',
        }),
        ('packInputPartInfos', packInputPartInfosSchema),
        ('packFormationParameters', packformationparametersschema.packFormationParametersSchema),
        ('packContainerType', packContainerTypeSchema),
        ('dynamicGoalsGeneratorParameters', dynamicgoalsconfigschema.dynamicGoalsConfigSchema),
        ('targetMinBottomPaddingForInitialTransfer', binpickingparametersschema.targetMinBottomPaddingForInitialTransferSchema),
        ('targetMinSafetyHeightForInitialTransfer', binpickingparametersschema.targetMinSafetyHeightForInitialTransferSchema),
        ('distanceMeasurementInfo', distanceMeasurementInfoSchema.distanceMeasurementInfoSchema),
        ('constraintToolInfo', binpickingparametersschema.constraintToolInfoSchema),
        ('saveDynamicGoalGeneratorState', binpickingparametersschema.saveDynamicGoalGeneratorStateSchema),
        ('saveDynamicGoalGeneratorStateFailed', binpickingparametersschema.saveDynamicGoalGeneratorStateFailedSchema),
        ('savePackingState', {
            'type': 'boolean',
        }),
        ('unitMass', unitMassSchema),
        ('debuglevel', components.debuglevel),
        ('packLocationName', {
            '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
            'type': 'string',
            'deprecated': True,
        }),
    ]),
}

hasDetectionObstaclesParametersSchema = {
    'type': 'object',
    'properties': OrderedDict([
        ('minDetectionImageTimeMS', {
            'type': 'integer',
        }),
        ('detectionInfos', detectionInfosSchema),
        ('pickLocationInfo', binpickingparametersschema.pickLocationInfoSchema),
        ('placeLocationInfos', binpickingparametersschema.placeLocationInfosSchema),
        ('packLocationInfo', packLocationInfoSchema),
        ('predictDetectionInfo', predictDetectionInfoSchema),
        ('useLocationState', {
            'type': 'boolean',
        }),
        ('sourcecontainername', {
            'type': 'string',
        }),
        ('bodyOcclusionInfos', {
            'type': 'array',
            'items': {
                'type': 'object',
            }
        }),
        ('forceWaitDestContainer', {
            'type': 'boolean'
        }),
        ('waitUpdateStampTimeout', {
            'type': 'integer',
        }),
        ('manipname', {
            'type': 'string',
        }),
        ('constraintToolDirection', components.constraintToolDirectionSchema),
        ('constraintToolOptions', {
            'type': 'integer',
        }),
        ('envclearance', components.envclearance),
        ('cameraOcclusionOffset', {
            'type': 'number',
        }),
        ('robotspeedmult', components.robotspeedmult),
        ('robotaccelmult', components.robotaccelmult),
        ('executionFilterFactor', {
            'type': 'number',
        }),
        ('executionConnectingTrajDecelMult', {
            'type': 'number',
        }),
        ('executionConnectingTrajReverseMult', {
            'type': 'number',
        }),
        ('executionReverseRecoveryDistance', {
            'type': 'number',
        }),
        ('locationCollisionInfos', components.locationCollisionInfos),
        ('destSensorSelectionInfos', destSensorSelectionInfosSchema),
        ('finalPlanMode', {
            'type': 'string',
        }),
        ('pathPlannerParameters', binpickingparametersschema.pathPlannerParametersSchema),
        ('jittererParameters', binpickingparametersschema.jittererParametersSchema),
        ('forceTorqueBasedEstimatorParameters', binpickingparametersschema.forceTorqueBasedEstimatorParametersSchema),
        ('smootherParameters', binpickingparametersschema.smootherParametersSchema),
        ('homePositionConfigurationName', {
            'type': 'string',
        }),
        ('cycleStartPositionConfigurationName', {
            'type': 'string',
        }),
        ('recoveryPositionConfigurationName', {
            'type': 'string',
        }),
        ('finalPlanPositionConfigurationName', {
            'type': 'string',
        }),
        ('ignoreStartPosition', ignoreStartPositionSchema),
        ('forceMoveToFinish', forceMoveToFinishSchema),
        ('ignoreFinishPosition', {
            'type': 'boolean',
        }),
        ('ignoreFinishPositionUnlessPackFormationComplete', {
            'type': 'boolean',
        }),
        ('justInTimeToolChangePlanning', {
            'type': 'object',
            'properties': {
                'toolNames': {
                    'type': 'array',
                    'items': {
                        'type': 'string',
                    }
                }
            },
        }),
        ('constraintDuringGrabbingToolDirection', components.constraintToolDirectionSchema),
        ('maxGrabbingManipSpeed', {
            'type': 'number',
        }),
        ('maxGrabbingManipAccel', {
            'type': 'number',
        }),
        ('maxFreeManipSpeed', {
            'type': 'number',
        }),
        ('maxFreeManipAccel', {
            'type': 'number',
        }),
        ('constraintToolInfo', binpickingparametersschema.constraintToolInfoSchema),
    ]),
}

startPackFormationComputationThreadParametersSchema = MergeDicts(
    [
        hasDetectionObstaclesParametersSchema,
        {
            'properties': OrderedDict([
                ('debuglevel', components.debuglevel),
                ('robotname', components.robotname),
                ('toolname', components.toolname),
                ('executionmode', {
                    'type': 'string',
                }),
                ('unit', components.unit),
                ('destcontainernames', destcontainernamesSchema),
                ('destcontainername', {
                    'type': 'string',
                }),
                ('containername', {
                    'type': 'string',
                }),
                ('packLocationInfo', packLocationInfoSchema),
                ('locationName', {
                    'type': 'string',
                }),
                ('packInputPartInfos', packInputPartInfosSchema),
                ('packFormationParameters', packformationparametersschema.packFormationParametersSchema),
                ('packContainerType', {
                    'type': 'string',
                }),
                ('dynamicGoalsGeneratorParameters', dynamicgoalsconfigschema.dynamicGoalsConfigSchema),
                ('targetMinBottomPaddingForInitialTransfer', binpickingparametersschema.targetMinBottomPaddingForInitialTransferSchema),
                ('targetMinSafetyHeightForInitialTransfer', binpickingparametersschema.targetMinSafetyHeightForInitialTransferSchema),
                ('distanceMeasurementInfo', distanceMeasurementInfoSchema.distanceMeasurementInfoSchema),
                ('constraintToolInfo', binpickingparametersschema.constraintToolInfoSchema),
                ('checkObstacleNames', checkObstacleNamesSchema),
                ('saveDynamicGoalGeneratorState', binpickingparametersschema.saveDynamicGoalGeneratorStateSchema),
                ('saveDynamicGoalGeneratorStateFailed', binpickingparametersschema.saveDynamicGoalGeneratorStateFailedSchema),
                ('savePackingState', {
                    'type': 'boolean',
                }),
                ('unitMass', unitMassSchema),
                ('csvHeader', csvHeaderDeprecatedSchema),
                ('packContainerId', {
                    '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
                    'type': 'string',
                    'deprecated': True,
                }),
                ('packLocationName', {
                    '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
                    'type': 'string',
                    'deprecated': True,
                }),
                ('containerNameTemplate', {
                    '$commend': 'Ignored by the server. Only passed because its is mixed up with useful parameters.',
                    'type': 'string',
                    'deprecated': True,
                }),
            ]),
        },
    ],
    deepcopy=True,
)[0]
