# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from . import _
from mujincommon.dictutil import MergeDicts

from . import components

from mujinbinpickingmanager.schema import binpickingparametersschema  # TODO(felixvd): Move into this repository
from mujinbinpickingmanager.schema import destGoalsSchema  # TODO(felixvd): Move into this repository


regionname = {
    'description': _('Name of the region of the objects.'),
    'mapsTo': 'containername',
    'type': 'string',
}

# TODO(andriy.logvin): Remove after these changes are landed to schema in https://git.mujin.co.jp/dev/binpickingui/-/merge_requests/2411
pieceInspectionInfoSchema = {
    "typeName": "PieceInspectionInfo",
    "title":_("Piece Inspection Info"),
    "description":_("Piece inspection settings at the middest"),
    "type": "object",
    "properties": {
        "expectedIOValue": {
            "title": _("Expected Piece Inspection IO Value"),
            "description": _("The expected value of the IO specified by 'ioName'."),
            "type": "number",
            "tags":["motion", "medium", "si"]
        },
        "ioCheckStartDelay": {
            "title": _("Piece Inspection IO Check Start Delay"),
            "description": _("seconds. Sometimes it takes time to read the piece inspection IO value, and this config compensates for the delay. The value should be smaller than midDestWaitTime."),
            "type": "number",
            "tags":["motion", "advanced", "si"]
        },
        "ioName": {
            "title": _("Piece Inspection IO Name"),
            "description": _("The IO name to check its value during piece inspection. 'expectedIOValue' should also be set."),
            "type": "string",
            "tags":["motion", "medium", "si"]
        },
        "use":{
            "title": _("Use Piece Inspection Info"),
            "description": _("If True and the 'moveToMidDest' is also True, then does piece inspection at the mid dest specified by 'midDestIkparamNames' and 'midDestCoordType' for 'midDestWaitTime'."),
            "type": "boolean",
            "default": False,
            "tags":["motion", "medium", "si"]
        }
    },
    "tags":["motion", "medium", "si"]
}

binpickingParametersSchema= MergeDicts(
    [
        binpickingparametersschema.binpickingParametersSchema,
        {
            'properties': {
                'jittererParametersOld': {  # In configs but not used in the actual code.
                    'type': 'object',
                    'deprecated': True,
                    'properties': {
                        'maxJitterLinkDist': {
                            'type': 'number',
                        }
                    }
                },
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
                'destcontainernames': {  # In 20220127_binpicking.py migrated to destContainerInfo
                    'type': 'array',
                    'items': {
                        'type': 'string'
                    }
                },
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
                'gripperInfo0': {  # In conf but it looks suspicious and I don't know if it should be added
                    'type': 'object',
                    'properties': {
                        'airchannels': {
                            'type': 'array',
                            'items': {
                                'type': 'object',
                                'properties': {
                                    'controlType': {
                                        'type': 'string',
                                    },
                                    'description': {
                                        'type': 'string',
                                    },
                                    'haveValve': {
                                        'type': 'boolean',
                                    },
                                    'maxIncidenceAngle': {
                                        'type': 'integer',
                                    },
                                    'maxWeights': {
                                        'type': 'array',
                                        'items': {
                                            'type': 'number'
                                        }
                                    },
                                    'suctionCupPartTypes': {
                                        'type': 'array',
                                        'items': {
                                            'type': 'string'
                                        }
                                    },
                                    'useLinkNames': {
                                        'type': 'array',
                                        'items': {
                                            'type': 'string'
                                        }
                                    },
                                }
                            }
                        },
                        'gripperModelId': {
                            'type': 'string',
                        },
                        'use': {
                            'type': 'boolean',
                        }
                    }
                },
                'forceMoveToFinish': {  # In conf but not in binpiskingparameters
                    'description': _('If True, then the robot will add a finish position to the cycle even if "finish" is not present, unless "ignoreFinishPosition" is True in the order cycle command. "ignoreFinishPosition" overrides this parameter.'),
                    'type': 'boolean',
                },
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
                'detectionInfos': {
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
                },
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
                'ignoreStartPosition': {  # In conf but not in binpiskingparameters
                    'description': _('True if the robot should ignore going to start position'),
                    'type': 'boolean',
                },
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
                'destSensorSelectionInfos': {
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
                'destdepartoffsetdir': components.departoffsetdir,  # Migrated in 20230505_approachDepartOffsets.py
                'deleteTargetWhenPlacedInDest': {
                    'type': ['string', 'boolean'],
                    'enum': ['DeleteInAll', 'KeepInAll', True, False],
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
                        "edgedetectorThresh": {
                            '$comment': 'Deprecated 2023/07/10.',
                            "title": _("Edge detector threshold"),
                            "description": _("discrete gradient, threshold for magnitude of gradient depth map image for determining edges"),
                            "type": "number",
                            "minimum": 0,
                            "maximum": 1,
                            "default": 0.1,
                            "tags":["advanced", "si", "dynamicGoals", "target"]
                        },
                        "intToolXYSize": {
                            '$comment': 'Deprecated 2019/05/23.',
                            "title": _("voxels, tool XY size"),
                            "description": _("The target will be grabbed by a tool when it is placed inside the container. By specifying the tool XY size, can assure that the target will not be placed too close to walls so that it is impossible for the tool to place."),
                            "type": "array",
                            "minItems": 2,
                            "maxItems": 2,
                            "items" :[
                                {"title": _("x"), "type":"integer", "default":0 },
                                {"title": _("y"), "type":"integer", "default":0 }
                            ],
                            "additionalItems": False,
                            "tags":["basic", "si", "dynamicGoals"]
                        },
                        "minCOMEdgeDistance": {
                            '$comment': 'Deprecated 2018/03/05, use jitterCOMRatioOffset.',
                            "title": _("Minimum COM Distance from Edge"),
                            "description": _("mm, the minimum allowed distance of the placed item COM from the edge of the supporting convex hull region under it"),
                            "type": "number",
                            "minimum": 0,
                            "default": 5,
                            "tags":["medium", "si", "dynamicGoals","target"]
                        },
                        "supportingWallTargetHeightRatio": {
                            '$comment': 'Deprecated in schema 20190523.',
                            "title": _("Supporting wall target height ratio"),
                            "description": _("ratio, how much of the box side has to be near the wall or another box."),
                            "type": "number",
                            "minimum": 0.0,
                            "default": 0.3,
                        },
                    }
                },
                'saveDynamicGoalGeneratorState': {
                    'type': 'boolean',
                    'enum': [0, 1, False, True],
                },
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
