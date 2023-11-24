# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from . import _
from mujincommon.dictutil import MergeDicts

from . import components

from mujinbinpickingmanager.schema import binpickingparametersschema  # TODO(felixvd): Move into this repository


regionname = {
    'description': _('Name of the region of the objects.'),
    'mapsTo': 'containername',
    'type': 'string',
}

binpickingParametersSchema= MergeDicts(
    [
        binpickingparametersschema.binpickingParametersSchema,
        {
            'properties': {
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
                'departoffsetdir': {  # Moved to graspDepartOffsetDir in 20220412_planningcommon_initial.py
                    'description': _('Departure offset direction. mm (x,y,z)'),
                    'type': 'array',
                    'minItems': 3,
                    'maxItems': 3,
                    'prefixItems': [
                        {'title': _('x'), 'type': 'number', 'default': 0},
                        {'title': _('y'), 'type': 'number', 'default': 0},
                        {'title': _('z'), 'type': 'number', 'default': 0}
                    ],
                    'additionalItems': False,
                },
                'itlParameters': {  # In conf but not in binpiskingparameters
                    'type': 'object',
                    'properties': {
                        'numTrajectoryBuffer': {
                            'type': 'integer',
                        },
                        'saveConcatenateTrajectoryLog': {
                            'type': 'boolean',
                        },
                        'saveFilterTrajectoryLog': {
                            'type': 'boolean',
                        },
                        'saveRobotFeedbackLog': {
                            'type': 'boolean',
                        },
                        'savetrajectorylog': {
                            'type': 'boolean',
                        },
                    }
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
                'destdepartoffsetdir': {  # Migrated in 20230505_approachDepartOffsets.py
                    'description': _('Departure offset direction. mm (x,y,z)'),
                    'type': 'array',
                    'minItems': 3,
                    'maxItems': 3,
                    'prefixItems': [
                        {'title': _('x'), 'type': 'number', 'default': 0},
                        {'title': _('y'), 'type': 'number', 'default': 0},
                        {'title': _('z'), 'type': 'number', 'default': 0}
                    ],
                    'additionalItems': False,
                },
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
            },
        },
    ],
    deepcopy=True,
)[0]
