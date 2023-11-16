# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

from . import components
from . import components_realtimerobot
from mujinbinpickingmanager.schema import binpickingparametersschema  # TODO(felixvd): Move into this repository


services = [
    ('GetJointValues', {
        'description': _('Gets the current robot joint values'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    OrderedDict([
                                        ('executetimeout', MergeDicts(
                                            [
                                                components_realtimerobot.executetimeout,
                                                {
                                                    'default': 10,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('unit', components.unit),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('currentjointvalues', {
                    'description': _('Current joint values.'),
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('MoveToolLinear', {
        'description': _('Moves the tool linearly in cartesian (3D) space.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('ignoreGrabbingTarget', components_realtimerobot.moveJointsParameters['ignoreGrabbingTarget']),
                                        ('currentlimitratios', components_realtimerobot.moveJointsParameters['currentlimitratios']),
                                        ('instobjectname', components_realtimerobot.Internal_MoveToParameters['instobjectname']),
                                        ('ikparamname', components_realtimerobot.Internal_MoveToParameters['ikparamname']),
                                        ('execute', components_realtimerobot.moveJointsParameters['execute']),
                                        ('moveStraightParams', components_realtimerobot.moveStraightParams),
                                        ('goaltype', MergeDicts(
                                            [
                                                components_realtimerobot.goaltype,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('goals', MergeDicts(
                                            [
                                                components_realtimerobot.goals,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('robotspeed', components.robotspeed),
                                        ('toolname', MergeDicts(
                                            [
                                                components.toolname,
                                                {
                                                    'description': _('Name of the manipulator. Default: self.toolname'),
                                                    'type': 'string',
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('workmaxdeviationangle', {
                                            'description': _('How much the tool tip can rotationally deviate from the linear path. In deg.'),
                                            'type': 'number',
                                        }),
                                        ('workspeed', components_realtimerobot.workspeed),
                                        ('workaccel', components_realtimerobot.workaccel),
                                        ('worksteplength', {
                                            'description': _('Discretization for planning MoveHandStraight, in seconds.'),
                                            'type': 'number',
                                        }),
                                        ('plannername', {
                                            'type': 'string',
                                        }),
                                        ('numspeedcandidates', {
                                            'description': _('If speed/accel are not specified, the number of candiates to consider'),
                                            'type': 'integer',
                                        }),
                                        ('workminimumcompletetime', {
                                            'deprecated': True,
                                            'description': _('Unused. Set to trajduration - 0.016s. EMU_MUJIN example requires at least this much'),
                                            'type': 'number',
                                        }),
                                        ('workminimumcompleteratio', {
                                            'deprecated': True,
                                            'description': _('Unused. In case the duration of the trajectory is now known, can specify in terms of [0,1]. 1 is complete everything.'),
                                            'type': 'number',
                                        }),
                                        ('workignorefirstcollisionee', {
                                            'description': _('time, necessary in case initial is in collision, has to be multiples of step length?'),
                                            'type': 'number',
                                        }),
                                        ('workignorelastcollisionee', {
                                            'description': _('time, necessary in case goal is in collision, has to be multiples of step length?'),
                                            'type': 'number',
                                        }),
                                        ('workignorefirstcollision', {
                                            'type': 'number',
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('MoveToHandPosition', {
        'description': _('Computes the inverse kinematics and moves the manipulator to any one of the goals specified.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    components_realtimerobot.moveJointsParameters,
                                    OrderedDict([
                                        ('minimumgoalpaths', components_realtimerobot.minimumgoalpaths),
                                        ('chuckgripper', {
                                            'type': 'boolean',
                                        }),
                                        ('instobjectname', components_realtimerobot.Internal_MoveToParameters['instobjectname']),
                                        ('ikparamname', components_realtimerobot.Internal_MoveToParameters['ikparamname']),
                                        ('ikparamoffset', components_realtimerobot.Internal_MoveToParameters['ikparamoffset']),
                                        ('pathPlannerParameters', {
                                            'properties': components_realtimerobot.pathPlannerParameters,
                                            'type': 'object',
                                        }),
                                        ('smootherParameters', {
                                            'properties': components_realtimerobot.smootherParameters,
                                            'type': 'object',
                                        }),
                                        ('moveStraightParams', components_realtimerobot.moveStraightParams),
                                        ('goaltype', MergeDicts(
                                            [
                                                components_realtimerobot.goaltype,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('goals', MergeDicts(
                                            [
                                                components_realtimerobot.goals,
                                                {
                                                    'description': _('Flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]'),
                                                    'isRequired': True,
                                                    'type': 'array',
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('toolname', components.toolname),
                                        ('envclearance', components.envclearance),
                                        ('closegripper', {
                                            'default': 0,
                                            'description': _('Whether to close gripper once the goal is reached. Boolean value represented by 0 or 1.'),
                                            'type': 'integer',
                                        }),
                                        ('robotspeed', components.robotspeed),
                                        ('robotaccelmult', components.robotaccelmult),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('UpdateObjects', {
        'description': _('Updates objects in the scene with the envstate'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.CanAcceptUpdateOnLocationParameters,
                                    OrderedDict([
                                        ('targetname', components.targetname),
                                        ('object_uri', MergeDicts(
                                            [
                                                components.targetname,
                                                {
                                                    'description': _('Same as objectname, but in a Mujin URI format, e.g.: mujin:/OBJECTNAME.mujin.dae'),
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('detectionResultState', {
                                            'description': _('Information about the detected objects (received from detectors)'),
                                            'type': 'object',
                                        }),
                                        ('targetUpdateNamePrefix', {
                                            'type': 'string',
                                        }),
                                        ('cameranames', {
                                            'items': {
                                                'type': 'string',
                                            },
                                            'type': 'array',
                                        }),
                                        ('countOverlappingPoints', {
                                            'type': 'boolean',
                                        }),
                                        ('overlapUpAxis', components.Vector3Schema),
                                        ('zthresholdmult', {
                                            'type': 'number',
                                        }),
                                        ('addUnpickableRegionAcrossShortEdgeDist', {
                                            'type': 'boolean',
                                        }),
                                        ('sizeRoundUp', {
                                            'description': _('If False, then round down. (Default: True)'),
                                            'type': 'boolean',
                                        }),
                                        ('sizePrecisionXYZ', binpickingparametersschema.binpickingParametersSchema['properties']['randomBoxInfo']['properties']['sizePrecisionXYZ']),
                                        ('points', {
                                            'description': _('The point cloud passed in along with the detection results. Used in selective cases to count point overlap of random box.'),
                                            'items': {
                                                'type': 'number',
                                            },
                                            'type': 'array',
                                        }),
                                        ('pointsize', {
                                            'description': _('Size of points in the point cloud.'),
                                            'type': 'number',
                                        }),
                                        ('pointcloudid', {
                                            'type': 'string',
                                        }),
                                        ('containerName', {
                                            'description': _('Name of the container to update. Requires locationName to be set. If containerName is empty, will use the container in locationName.'),
                                            'type': 'string',
                                        }),
                                        ('isFromStateSlaveNotify', {
                                            'type': 'boolean',
                                        }),
                                        ('imageStartTimeStampMS', {
                                            'type': 'integer',
                                        }),
                                        ('imageEndTimeStampMS', {
                                            'type': 'integer',
                                        }),
                                        ('pointCloudSensorTimeStampMS', {
                                            'type': 'integer',
                                        }),
                                        ('belowBoxOverlap', binpickingparametersschema.targetOverlapConstraintInfoSchema['properties']['belowBoxOverlap']),
                                        ('ignoreOverlapPointsFromWall', binpickingparametersschema.targetOverlapConstraintInfoSchema['properties']['ignoreOverlapPointsFromWall']),
                                        ('ignoreOverlapPointsFromNearbyTargets', binpickingparametersschema.targetOverlapConstraintInfoSchema['properties']['ignoreOverlapPointsFromNearbyTargets']),
                                        ('castPointCloudShadowFromCamera', {
                                            'description': _('If True, bottom parts of pointcloud obstacle are generated by casting shadow from camera. otherwise, vertical down (-z).'),
                                            'type': 'boolean',
                                        }),
                                        ('pointsProjectedDirection', MergeDicts(
                                            [
                                                components.Vector3Schema,
                                                {
                                                    'description': _('The negative direction in which the points were projected when creating the obstacles. If specified, then take into account when computing the overlap. When container up is +Z, then pointsProjectedDirection will be (0,0,1).'),
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('randomBoxOrigin', binpickingparametersschema.binpickingParametersSchema['properties']['randomBoxInfo']['properties']['randomBoxOrigin']),
                                        ('rollStepDegree', binpickingparametersschema.binpickingParametersSchema['properties']['randomBoxInfo']['properties']['rollStepDegree']),
                                        ('clampToContainer', {
                                            'description': _('If True, crop to container dimensions.'),
                                            'type': 'boolean',
                                        }),
                                        ('medianFilterHalfSize', {
                                            'description': _('If clampcontainer is True, this is used for filtering.'),
                                            'type': 'number',
                                        }),
                                        ('useEmptyRegionForCropping', {
                                            'description': _('If clampcontainer is True, this is used for filtering.'),
                                            'type': 'boolean',
                                        }),
                                        ('ioSignalsInfo', components_realtimerobot.ioSignalsInfo),
                                        ('addPointOffsetInfo', {
                                            'type': 'object',
                                        }),
                                        ('envstate', {
                                            'description': _("A list of dictionaries for each instance object in world frame. Quaternion is specified in w,x,y,z order. e.g. [{'name': 'target_0', 'translation_': [1,2,3], 'quat_': [1,0,0,0], 'object_uri':'mujin:/asdfas.mujin.dae'}, {'name': 'target_1', 'translation_': [2,2,3], 'quat_': [1,0,0,0]}]"),
                                            'isRequired': True,
                                        }),
                                        ('state', {
                                            'type': 'object',
                                        }),
                                        ('unit', components.unit),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('Grab', {
        'description': _('Grabs an object with tool'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', MergeDicts(
                                    [
                                        components.targetname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('toolname', components.toolname),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('Release', {
        'description': _('Releases a grabbed object.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', MergeDicts(
                                    [
                                        components.targetname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetGrabbed', {
        'description': _('Gets the names of the objects currently grabbed'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('names', {
                    'description': _('Names of the grabbed object.'),
                    'items': {
                        'type': 'string',
                    },
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('GetTransform', {
        'description': _('Gets the transform of an object'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', {
                                    'description': _('OpenRave Kinbody name'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('connectedBodyName', {
                                    'default': '',
                                    'description': _('OpenRave connected body name'),
                                    'type': 'string',
                                }),
                                ('linkName', {
                                    'default': '',
                                    'description': _('OpenRave link name'),
                                    'type': 'string',
                                }),
                                ('geometryName', {
                                    'default': '',
                                    'description': _('OpenRave geometry id name'),
                                    'type': 'string',
                                }),
                                ('geometryPk', {
                                    'default': '',
                                    'description': _('OpenRave geometry primary key (pk)'),
                                    'type': 'string',
                                }),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('Transform of the object.'),
            'properties': OrderedDict([
                ('translation', components.translation),
                ('rotationmat', {
                    'description': _('E.g. [[1,0,0],[0,1,0],[0,0,1]]'),
                    'items': {
                        'items': {
                            'type': 'number',
                        },
                        'type': 'array',
                    },
                    'type': 'array',
                }),
                ('quaternion', components.quaternion),
            ]),
            'type': 'object',
        },
    }),
    ('GetLinkParentInfo', {
        'description': _('Gets the parent link transform and name.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('objectName', {
                                    'description': _('OpenRave Kinbody name.'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('linkName', {
                                    'description': _('OpenRave link name.'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('name', {
                    'type': 'string',
                }),
                ('translation', {
                    'type': 'array',
                }),
                ('rotationmat', {
                    'type': 'array',
                }),
                ('quaternion', {
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('SetTransform', {
        'description': _('Sets the transform of an object. Rotation can be specified by either quaternion or rotation matrix.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', MergeDicts(
                                    [
                                        components.targetname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('translation', MergeDicts(
                                    [
                                        components.translation,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('unit', components.unit),
                                ('rotationmat', components_realtimerobot.rotationmat),
                                ('quaternion', components.quaternion),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetOBB', {
        'description': _('Get the oriented bounding box (OBB) of object.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', MergeDicts(
                                    [
                                        components.targetname,
                                        {
                                            'description': _('Name of the object'),
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('unit', components.unit),
                                ('linkname', components_realtimerobot.linknameOBB),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('A dictionary describing the OBB of the object.'),
            'properties': OrderedDict([
                ('extents', {}),
                ('boxLocalTranslation', {}),
                ('originalBodyTranslation', {}),
                ('quaternion', {}),
                ('rotationmat', {}),
                ('translation', {}),
            ]),
            'type': 'object',
        },
    }),
    ('GetInnerEmptyRegionOBB', {
        'description': _('Get the inner empty oriented bounding box (OBB) of a container.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', MergeDicts(
                                    [
                                        components.targetname,
                                        {
                                            'description': _('Name of the object'),
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('linkname', components_realtimerobot.linknameOBB),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('A dictionary describing the OBB of the object.'),
            'properties': OrderedDict([
                ('extents', {}),
                ('boxLocalTranslation', {}),
                ('originalBodyTranslation', {}),
                ('quaternion', {}),
                ('rotationmat', {}),
                ('translation', {}),
            ]),
            'type': 'object',
        },
    }),
    ('GetInstObjectAndSensorInfo', {
        'description': _('Returns information about the inst objects and sensors that are a part of those inst objects.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('instobjectnames', {
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                                ('sensornames', {
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                                ('unit', components.unit),
                                ('ignoreMissingObjects', {
                                    'description': _('If False, will raise an error if the object is not found in the scene. Default: True.'),
                                    'type': 'boolean',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetInstObjectInfoFromURI', {
        'description': _('Opens a URI and returns info about the internal/external and geometry info from it.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('objecturi', {
                                    'type': 'string',
                                }),
                                ('unit', components.unit),
                                ('instobjectpose', {
                                    'description': _('Pose to be assigned to the retrieved object. 7-element list'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetAABB', {
        'description': _('Gets the axis-aligned bounding box (AABB) of an object.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', MergeDicts(
                                    [
                                        components.targetname,
                                        {
                                            'description': _('Name of the object'),
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('unit', components.unit),
                                ('linkname', components_realtimerobot.linknameAABB),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('AABB of the object.'),
            'properties': OrderedDict([
                ('pos', {
                    'description': _('E.g. [1000,400,100]'),
                    'type': 'array',
                }),
                ('extents', {
                    'description': _('E.g. [100,200,50]'),
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('SetLocationTracking', {
        'description': _('Resets the tracking of specific containers'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('cycleIndex', {
                                    'description': _('The cycle index to track the locations for'),
                                }),
                                ('locationReplaceInfos', {
                                    'description': _('A dict that should have the keys: name, containerDynamicProperties, rejectContainerIds, uri, pose, cycleIndex'),
                                }),
                                ('removeLocationNames', components_realtimerobot.removeLocationNames),
                                ('minRobotBridgeTimeStampUS', {
                                    'description': _('The minimum expected time stamp.'),
                                    'type': 'integer',
                                }),
                                ('dynamicObstacleBaseName', {
                                    'type': 'string',
                                }),
                                ('targetUpdateBaseName', {
                                    'type': 'string',
                                }),
                                ('ioSignalsInfo', components_realtimerobot.ioSignalsInfo),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ResetLocationTracking', {
        'description': _('Resets tracking updates for locations'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('resetAllLocations', {
                                    'description': _('If True, then will reset all the locations'),
                                    'type': 'boolean',
                                }),
                                ('resetLocationName', {
                                    'description': _('Resets only the location with matching name'),
                                    'type': 'string',
                                }),
                                ('resetLocationNames', {
                                    'description': _('Resets only locations with matching name'),
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                                ('checkIdAndResetLocationName', {
                                    'description': _('(locationName, containerId) - only reset the location if the container id matches'),
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('clearedLocationNames'),
            'properties': OrderedDict([
                ('clearedLocationNames', {
                    'description': _('clearedLocationNames'),
                    'type': 'object',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('GetLocationTrackingInfos', {
        'description': _('Gets the active tracked locations'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('activeLocationTrackingInfos'),
            'properties': OrderedDict([
                ('activeLocationTrackingInfos', {
                    'description': _('activeLocationTrackingInfos'),
                    'type': 'object',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('UpdateLocationContainerIdType', {
        'description': _('Resets the tracking of specific containers'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('locationName', {
                                    'description': _('Name of the location the container is in'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('containerName', {
                                    'description': _('Name of the container'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('containerId', {
                                    'description': _('ID of the container'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('containerType', {
                                    'description': _('Type of the container'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('trackingCycleIndex', {
                                    'description': _('If specified, then the cycle with same cycleIndex will update location tracking in the same call.'),
                                }),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ResetLocationTrackingContainerId', {
        'description': _('Resets the containerId of self._activeLocationTrackingInfos if it matches checkContainerId.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('locationName', {
                                    'description': _('The name of the location that may be reset.'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('checkContainerId', {
                                    'description': _('If checkContainerId is specified and not empty and it matches the current containerId of the tracking location, then reset the current tracking location'),
                                    'isRequired': True,
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RemoveObjectsWithPrefix', {
        'description': _('Removes objects with prefix.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.CanAcceptUpdateOnLocationParameters,
                                    OrderedDict([
                                        ('prefix', {
                                            'deprecated': True,
                                            'description': _(''),
                                            'type': 'string',
                                        }),
                                        ('removeNamePrefixes', components_realtimerobot.removeNamePrefixes),
                                        ('removeLocationNames', components_realtimerobot.removeLocationNames),
                                        ('doRemoveOnlyDynamic', components_realtimerobot.doRemoveOnlyDynamic),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('removedBodyNames', {
                    'description': _('Key for the removed object names'),
                }),
            ]),
            'type': 'object',
        },
    }),
    ('GetTrajectoryLog', {
        'description': _('Gets the recent trajectories executed on the binpicking server. The internal server keeps trajectories around for 10 minutes before clearing them.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('saverawtrajectories', {
                                    'description': _('If True, will save the raw trajectories.'),
                                    'type': 'boolean',
                                }),
                                ('startindex', {
                                    'description': _('Start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last. Default: 0'),
                                    'type': 'integer',
                                }),
                                ('num', {
                                    'description': _('Number of trajectories from startindex to return. If 0, will return all the trajectories starting from startindex. Default: 0'),
                                    'type': 'integer',
                                }),
                                ('includejointvalues', {
                                    'default': False,
                                    'description': _('If True, will include timedjointvalues. If False, will just give back the trajectories.'),
                                    'type': 'boolean',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('total', {
                    'type': 'integer',
                }),
                ('trajectories', {
                    'items': {
                        'properties': OrderedDict([
                            ('timestarted', {
                                'type': 'integer',
                            }),
                            ('name', {
                                'type': 'string',
                            }),
                            ('numpoints', {
                                'type': 'integer',
                            }),
                            ('duration', {
                                'type': 'number',
                            }),
                            ('timedjointvalues', {
                                'description': _('A list of joint values and the trajectory time. For a 3DOF robot sampled at 0.008s, this is: [J1, J2, J3, 0, J1, J2, J3, 0.008, J1, J2, J3, 0.016, ...]'),
                                'type': 'array',
                            }),
                        ]),
                        'type': 'object',
                    },
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('ChuckGripper', {
        'description': _('Chucks the manipulator'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('robotname', components.robotname),
                                        ('grippername', components_realtimerobot.grippername),
                                        ('toolname', components.toolname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('UnchuckGripper', {
        'description': _('Unchucks the manipulator and releases the target'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('robotname', components.robotname),
                                        ('grippername', components_realtimerobot.grippername),
                                        ('targetname', MergeDicts(
                                            [
                                                components.targetname,
                                                {
                                                    'description': _('Name of the target object.'),
                                                }
                                            ]
                                        )[0]),
                                        ('toolname', components.toolname),
                                        ('pulloutdist', {
                                            'description': _('Distance to move away along the tool direction after releasing.'),
                                            'type': 'number',
                                        }),
                                        ('deletetarget', {
                                            'description': _('If 1, removes the target object from the environment after releasing. (Default: 1)'),
                                            'type': 'integer',
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('CalibrateGripper', {
        'description': _('Goes through the gripper calibration procedure'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('robotname', components.robotname),
                                        ('grippername', components_realtimerobot.grippername),
                                        ('toolname', components.toolname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StopGripper', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('robotname', components.robotname),
                                        ('grippername', components_realtimerobot.grippername),
                                        ('toolname', components.toolname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('MoveGripper', {
        'description': _('Moves the chuck of the manipulator to a given value.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('grippername', components_realtimerobot.grippername),
                                        ('grippervalues', {
                                            'description': _('Target value(s) of the chuck.'),
                                            'isRequired': True,
                                            'items': {
                                                'type': 'number',
                                            },
                                            'type': 'array',
                                        }),
                                        ('robotname', components.robotname),
                                        ('toolname', components.toolname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ExecuteRobotProgram', {
        'description': _('Execute a robot specific program by name'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('robotProgramName', {
                                            'isRequired': True,
                                            'type': 'string',
                                        }),
                                        ('robotname', components.robotname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SaveScene', {
        'description': _('Saves the current scene to file'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('filename', {
                                    'description': _('e.g. /tmp/testscene.mujin.dae, if not specified, it will be saved with an auto-generated filename'),
                                    'type': 'string',
                                }),
                                ('preserveexternalrefs', {
                                    'description': _('If True, any bodies that are currently being externally referenced from the environment will be saved as external references.'),
                                    'type': 'boolean',
                                }),
                                ('externalref', {
                                    'description': _("If '*', then each of the objects will be saved as externally referencing their original filename. Otherwise will force saving specific bodies as external references."),
                                    'type': 'string',
                                }),
                                ('saveclone', {
                                    'deprecated': True,
                                    'description': _('If 1, will save the scenes for all the cloned environments'),
                                }),
                                ('saveReferenceUriAsHint', {
                                    'description': _('If True, use save the reference uris as referenceUriHint so that webstack does not get confused and deletes content'),
                                    'type': 'boolean',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _("The filename the scene is saved to, in a json dictionary, e.g. {'filename': '2013-11-01-17-10-00-UTC.dae'}"),
            'type': 'object',
        },
    }),
    ('SaveGripper', {
        'description': _('Separate gripper from a robot in a scene and save it.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    OrderedDict([
                                        ('robotname', MergeDicts(
                                            [
                                                components.robotname,
                                                {
                                                    'description': _('Name of the robot waiting to extract the hand from.'),
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('filename', {
                                            'description': _('File name to save on the file system. e.g. /tmp/robotgripper/mujin.dae'),
                                            'type': 'string',
                                        }),
                                        ('manipname', {
                                            'description': _('Name of the manipulator.'),
                                            'type': 'string',
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('MoveJointsToJointConfigurationStates', {
        'description': _('Moves the robot to desired joint angles specified in jointStates'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    components_realtimerobot.moveJointsParameters,
                                    OrderedDict([
                                        ('goalJointConfigurationStates', {
                                            'isRequired': True,
                                        }),
                                        ('robotname', components.robotname),
                                        ('startJointConfigurationStates', {}),
                                        ('jointStates', {
                                            'description': _("List[{'jointName':str, 'jointValue':float}]"),
                                            'type': 'array',
                                        }),
                                        ('jointindices', components_realtimerobot.jointindices),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('MoveJoints', {
        'description': _('Moves the robot to desired joint angles specified in jointvalues'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    components_realtimerobot.moveJointsParameters,
                                    OrderedDict([
                                        ('jointvalues', {
                                            'isRequired': True,
                                            'items': {
                                                'type': 'number',
                                            },
                                            'type': 'array',
                                        }),
                                        ('goaljoints', {
                                            'description': _('List of joint values to move to.'),
                                            'items': {
                                                'type': 'number',
                                            },
                                            'type': 'array',
                                        }),
                                        ('robotJointNames', {
                                            'description': _('List of corresponding joint names for jointvalues.'),
                                            'type': 'string',
                                        }),
                                        ('forceTorqueBasedEstimatorParameters', components_realtimerobot.forceTorqueBasedEstimatorParameters),
                                        ('jointindices', components_realtimerobot.jointindices),
                                        ('robotname', components.robotname),
                                        ('startvalues', components_realtimerobot.startvalues),
                                        ('robotProgramName', {
                                            'type': 'string',
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('MoveJointsToPositionConfiguration', {
        'description': _('Moves the robot to desired position configuration specified in positionConfigurationName'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    components_realtimerobot.moveJointsParameters,
                                    OrderedDict([
                                        ('startJointConfigurationStates', {
                                            'description': _('List of dicts for each joint.'),
                                            'items': {
                                                'type': 'object',
                                            },
                                            'type': 'array',
                                        }),
                                        ('robotProgramName', {
                                            'type': 'string',
                                        }),
                                        ('forceTorqueBasedEstimatorParameters', components_realtimerobot.forceTorqueBasedEstimatorParameters),
                                        ('positionConfigurationName', {
                                            'description': _('If specified, the name of position configuration to move to. If it does not exist, will raise an error.'),
                                            'type': 'string',
                                        }),
                                        ('positionConfigurationCandidateNames', {
                                            'description': _('If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot.'),
                                            'items': {
                                                'type': 'string',
                                            },
                                            'type': 'array',
                                        }),
                                        ('robotname', components.robotname),
                                        ('startvalues', components_realtimerobot.startvalues),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('Dictionary with keys goalPositionName and values goalConfiguration'),
            'type': 'object',
        },
    }),
    ('GetRobotBridgeIOVariables', {
        'description': _('Returns the data of the IO in ASCII hex as a string'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    OrderedDict([
                                        ('ioname', MergeDicts(
                                            [
                                                components_realtimerobot.ioname,
                                                {
                                                    'description': _('One IO name to read'),
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('ionames', MergeDicts(
                                            [
                                                components_realtimerobot.ionames,
                                                {
                                                    'description': _('A list of the IO names to read'),
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('robotname', components.robotname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgeIOVariables', {
        'description': _('Sets a set of IO variables in the robot bridge.\nThis should not lock self.env since it can happen during the runtime of a task and lock out other functions waiting in the queue.\n'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    OrderedDict([
                                        ('forceasync', {
                                            'type': 'boolean',
                                        }),
                                        ('iovalues', {
                                            'isRequired': True,
                                        }),
                                        ('robotname', components.robotname),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ComputeIkParamPosition', {
        'description': _('Given the name of a Kinbody, computes the manipulator (TCP) position in the Kinbody frame to generate values for an IKParameterization.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('robotname', components.robotname),
                                ('toolname', components.toolname),
                                ('unit', components.unit),
                                ('name', {
                                    'description': _('Name of the Kinbody (the robot).'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('jointvalues', MergeDicts(
                                    [
                                        components_realtimerobot.jointvalues,
                                        {
                                            'description': _("If given, the robot's joints are set to these values before calculating the manipulator (TCP) position. If not set, uses the current values."),
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('translation', components.translation),
                ('quaternion', components.quaternion),
                ('direction', MergeDicts(
                    [
                        components.Vector3Schema,
                        {
                            'description': _('The global direction of the manipulator (assuming that the direction of the manipulator is the positive Z-axis).'),
                        }
                    ],
                    deepcopy=True,
                )[0]),
                ('angleXZ', {
                    'type': 'number',
                }),
                ('angleYX', {
                    'type': 'number',
                }),
                ('angleZY', {
                    'type': 'number',
                }),
                ('angleX', {
                    'type': 'number',
                }),
                ('angleY', {
                    'type': 'number',
                }),
                ('angleZ', {
                    'type': 'number',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('ComputeIKFromParameters', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('targetname', components.targetname),
                                ('graspsetname', components.graspsetname),
                                ('ikparamnames', components_realtimerobot.ikparamnames),
                                ('limit', {
                                    'description': _('Number of solutions to return'),
                                    'type': 'number',
                                }),
                                ('useSolutionIndices', {
                                    'type': 'boolean',
                                }),
                                ('disabletarget', {
                                    'type': 'boolean',
                                }),
                                ('unit', components.unit),
                                ('randomBoxInfo', MergeDicts(
                                    [
                                        binpickingparametersschema.binpickingParametersSchema['properties']['randomBoxInfo'],
                                        {
                                            'description': _('Info structure for maintaining grasp parameters for random box picking. Used when picking up randomized boxes (targetIsRandomBox is True). Keys: usefaces, dictFacePriorities, boxDirAngle, toolTranslationOffsets'),
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('freeincvalue', {
                                    'description': _('The discretization of the free joints of the robot when computing ik.'),
                                    'type': 'number',
                                }),
                                ('freeinc', {
                                    'deprecated': True,
                                    'description': _('The discretization of the free joints of the robot when computing ik.'),
                                    'type': 'number',
                                }),
                                ('applyapproachoffset', {
                                    'type': 'boolean',
                                }),
                                ('inPlaneAngleDeviation', {
                                    'type': 'number',
                                }),
                                ('outOfPlaneAngleDeviation', {
                                    'type': 'number',
                                }),
                                ('searchfreeparams', {
                                    'type': 'boolean',
                                }),
                                ('returnClosestToCurrent', {
                                    'type': 'boolean',
                                }),
                                ('filteroptionslist', {
                                    'description': _('A list of filter option strings. Can be: CheckEnvCollisions, IgnoreCustomFilters, IgnoreEndEffectorCollisions, IgnoreEndEffectorEnvCollisions, IgnoreEndEffectorSelfCollisions, IgnoreJointLimits, IgnoreSelfCollisions. Overrides filteroptions.'),
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                                ('filteroptions', {
                                    'description': _('OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked'),
                                    'type': 'integer',
                                }),
                                ('robotname', components.robotname),
                                ('toolname', MergeDicts(
                                    [
                                        components.toolname,
                                        {
                                            'description': _('Tool name'),
                                            'type': 'string',
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('solutions', {
                    'description': _('Array of IK solutions (each of which is an array of DOF values), sorted by minimum travel distance and truncated to match the limit'),
                    'items': {
                        'type': 'object',
                    },
                    'type': 'array',
                }),
                ('errors', {
                    'description': _("If no solutions found, the field 'errors' will contain reasons for the failure."),
                    'items': {
                        'type': 'object',
                    },
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('ReloadModule', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': components_realtimerobot.Internal_SetRobotClientParameters,
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ShutdownRobotBridge', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': components_realtimerobot.Internal_SetRobotClientParameters,
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetRobotBridgeState', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    OrderedDict([
                                        ('ionames', components_realtimerobot.ionames),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ClearRobotBridgeError', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': components_realtimerobot.Internal_SetRobotClientParameters,
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgePause', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': components_realtimerobot.Internal_SetRobotClientParameters,
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgeResume', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': components_realtimerobot.Internal_SetRobotClientParameters,
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgeServoOn', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('isservoon', {
                                    'description': _('If True, turns servo on.'),
                                    'isRequired': True,
                                    'type': 'boolean',
                                }),
                                ('robotname', components.robotname),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgeLockMode', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('islockmode', {
                                    'description': _('If True, turns on Lock Mode. During Lock Mode, all communication with the physical robot is turned off and the hardware will not move.'),
                                    'isRequired': True,
                                    'type': 'boolean',
                                }),
                                ('robotname', components.robotname),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ResetSafetyFault', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgeControlMode', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('controlMode', {
                                    'description': _('The control mode to use, e.g. "Manual".'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetDynamicObjects', {
        'description': _('Get a list of dynamically added objects in the scene, from vision detection and physics simulation.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ComputeRobotConfigsForGraspVisualization', {
        'description': _('Returns robot configs for grasp visualization'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('approachoffset', {
                                    'type': 'number',
                                }),
                                ('departoffsetdir', components_realtimerobot.moveJointsParameters['departOffsetDir']),
                                ('departoffsetintool', {
                                    'description': _(' '),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('shadowrobotname', {
                                    'type': 'string',
                                }),
                                ('shadowrobottoolname', {
                                    'type': 'string',
                                }),
                                ('targetname', {
                                    'description': _("Target object's name."),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('graspname', {
                                    'description': _('Name of the grasp for which to visualize grasps.'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('robotname', MergeDicts(
                                    [
                                        components.robotname,
                                        {
                                            'type': 'string',
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('toolname', MergeDicts(
                                    [
                                        components.toolname,
                                        {
                                            'description': _('Name of the manipulator. (Default: "self.toolname")'),
                                            'type': 'string',
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ResetCacheTemplates', {
        'description': _('Resets any cached templates'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetRobotBridgeExternalIOPublishing', {
        'description': _('Enables publishing collision data to the robotbridge'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('enable', {
                                    'description': _('If True, collision data will be published to robotbridge.'),
                                    'isRequired': True,
                                    'type': 'boolean',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RestoreSceneInitialState', {
        'description': _('Restores the scene to the state on the filesystem'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('preserverobotdofvalues', {
                                    'description': _('A Boolean value represented by 0 or 1.'),
                                    'type': 'integer',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RunMotorControlTuningStepTest', {
        'description': _('Runs step response test on specified joint and returns result'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('jointName', MergeDicts(
                                    [
                                        components_realtimerobot.jointName,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('amplitude', MergeDicts(
                                    [
                                        components_realtimerobot.amplitude,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RunMotorControlTuningMaximulLengthSequence', {
        'description': _('Runs maximum length sequence test on specified joint and returns result'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', MergeDicts(
                                            [
                                                components_realtimerobot.jointName,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('amplitude', MergeDicts(
                                            [
                                                components_realtimerobot.amplitude,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RunMotorControlTuningDecayingChirp', {
        'description': _('runs chirp test on specified joint and returns result'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', MergeDicts(
                                            [
                                                components_realtimerobot.jointName,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('freqMax', MergeDicts(
                                            [
                                                components_realtimerobot.freqMax,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('amplitude', MergeDicts(
                                            [
                                                components_realtimerobot.amplitude,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RunMotorControlTuningGaussianImpulse', {
        'description': _('Runs Gaussian Impulse test on specified joint and returns result'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', MergeDicts(
                                            [
                                                components_realtimerobot.jointName,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('amplitude', MergeDicts(
                                            [
                                                components_realtimerobot.amplitude,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RunMotorControlTuningBangBangResponse', {
        'description': _('Runs bangbang trajectory in acceleration or jerk space and returns result'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', MergeDicts(
                                            [
                                                components_realtimerobot.jointName,
                                                {
                                                    'isRequired': True,
                                                },
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('amplitude', MergeDicts(
                                            [
                                                components_realtimerobot.amplitude,
                                                {
                                                    'isRequired': True,
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('RunDynamicsIdentificationTest', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetTimeToRunDynamicsIdentificationTest', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', components_realtimerobot.jointName),
                                        ('minJointAngle', {
                                            'description': _('The joint angle to start the dynamics identification test at.'),
                                            'type': 'number',
                                        }),
                                        ('maxJointAngle', {
                                            'description': _('The joint angle to finish the dynamics identification test at.'),
                                            'type': 'number',
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('CalculateTestRangeFromCollision', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', components_realtimerobot.jointName),
                                        ('unit', components.unit),
                                        ('envclearance', components.envclearance),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetMotorControlParameterSchema', {
        'description': _('Gets motor control parameter schema'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetMotorControlParameter', {
        'description': _("Gets motor control parameters as a name-value dict, e.g.: {'J1':{'KP':1}, 'J2':{'KV':2}}"),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', {
                                            'description': _('The name of the joint.'),
                                            'isRequired': True,
                                            'type': 'string',
                                        }),
                                        ('parameterName', {
                                            'isRequired': True,
                                            'type': 'string',
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetMotorControlParameters', {
        'description': _('Gets cached motor control parameters as name-value dict'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetMotorControlParameter', {
        'description': _('Sets motor control parameter'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components_realtimerobot.Internal_SetRobotClientParameters,
                                    components_realtimerobot.Internal_MoveCommandDecoratorParameters,
                                    OrderedDict([
                                        ('jointName', MergeDicts(
                                            [
                                                components_realtimerobot.jointName,
                                                {
                                                    'isRequired': True,
                                                    'type': 'string',
                                                }
                                            ],
                                            deepcopy=True,
                                        )[0]),
                                        ('parameterName', {
                                            'description': _('The name of the parameter to set.'),
                                            'isRequired': True,
                                            'type': 'string',
                                        }),
                                        ('parameterValue', {
                                            'description': _('The value to assign to the parameter.'),
                                            'isRequired': True,
                                        }),
                                    ])
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('IsProfilingRunning', {
        'description': _('Queries if profiling is running on planning'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StartProfiling', {
        'description': _('Start profiling planning'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('clocktype', {
                                    'default': 'cpu',
                                    'type': 'string',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StopProfiling', {
        'description': _('Stop profiling planning'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ReplaceBodies', {
        'description': _('Replaces bodies in the environment with new uris'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('bodieslist', {
                                    'description': _('Used as replaceInfos if the replaceInfos is not defined. Used for backwards compatibility only.'),
                                    'isRequired': True,
                                }),
                                ('replaceInfos', {
                                    'description': _('list of dicts with keys: name, uri, containerDynamicProperties'),
                                    'items': {
                                        'type': 'object',
                                    },
                                    'type': 'array',
                                }),
                                ('testLocationName', {
                                    'description': _('If specified, will test if the container in this location matches testLocationContainerId, and only execute the replace if it matches and testLocationContainerId is not empty.'),
                                    'type': 'string',
                                }),
                                ('testLocationContainerId', {
                                    'description': _('containerId used for testing logic with testLocationName'),
                                    'type': 'string',
                                }),
                                ('removeNamePrefixes', components_realtimerobot.removeNamePrefixes),
                                ('removeLocationNames', components_realtimerobot.removeLocationNames),
                                ('doRemoveOnlyDynamic', components_realtimerobot.doRemoveOnlyDynamic),
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetState', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': components_realtimerobot.Internal_SetRobotClientParameters,
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('EnsureSyncWithRobotBridge', {
        'description': _('Ensures that planning has synchronized with robotbridge data that is newer than syncTimeStampUS'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('syncTimeStampUS', {
                                    'description': _('us (microseconds, linux time) of the timestamp'),
                                    'isRequired': True,
                                    'type': 'integer',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ResetCachedRobotConfigurationState', {
        'description': _('Resets cached robot configuration (position of the robot) in the planning slave received from slave notification. Need to perform every time robot moved not from the task slaves.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict(),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
]

realtimeRobotSpec = {
    'info': {
        'title': _('RealtimeRobot'),
        'description': 'The RealtimeRobot API of the Mujin Planning Server.',
        'mujinspecformatversion': '0.0.1',
    },
    'services': OrderedDict(services),
}