# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

import sys
if sys.version_info[0] >= 3:
    raise ImportError("This file cannot be used until binpickingui is ported to python3, or we put the binpicking schemas in a separate module")

from mujinbinpickingmanager.schema import binpickingparametersschema  # TODO(felixvd): Fix this dependency


uriSchema = {
    'type': 'string',
    'semanticType': 'Uri',
}

constraintToolDirectionSchema = {
    'description': _('Contains 7 params: manipdir, globaldir, cosangle.'),
    'type': 'array',
    'minItems': 7,
    'maxItems': 7,
    'prefixItems': [
        {'title': _('manipdirX'), 'type': 'number'},
        {'title': _('manipdirY'), 'type': 'number'},
        {'title': _('manipdirZ'), 'type': 'number'},
        {'title': _('globaldirX'), 'type': 'number'},
        {'title': _('globaldirY'), 'type': 'number'},
        {'title': _('globaldirZ'), 'type': 'number'},
        {'title': _('cosangle'), 'type': 'number'},
    ],
}

debuglevel = {
    'description': _('Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE.'),
    'type': 'integer',
}

departoffsetdir = {
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
}

dynamicEnvironmentState = {
    'description': _('Dynamic environment state that allows the user to set/create objects in a particular state dynamically.'),
    'type': 'object',
    'additionalProperties': {
        'type': 'object',
        'properties': {
            'animate': {
                'description': _('Animate value is a update stamp, when it changes, we need to restart animation.'),
                'type': 'integer',
            },
            'boxFullSize': {
                'type': 'array',
                'minItems': 3,
                'maxItems': 3,
                'items': {
                    'type': 'number',
                }
            },
            'cloneOriginalBodyName': {
                'type': 'boolean',
            },
            'collision': {
                'type': 'boolean',
            },
            'dofvalues': {
                'type': 'array',
                'items': {
                    'type': 'number',
                }
            },
            'exclusive': {
                'type': 'boolean',
            },
            'grabbedby': {
                'type': 'array',
                'additionalItems': False,
                'maxItems': 2,
                'minItems': 2,
                'prefixItems': [
                    {
                        'type': 'string',
                    },
                    {
                        'type': 'string',
                    },
                ],
            },
            'iscreated': {
                'type': 'boolean',
            },
            'linkstates': {
                'type': 'object',
                'additionalProperties': {
                    'type': 'object'
                }
            },
            'linkenable': {
                'description': _('Link name.'),
                'type': 'string',
            },
            'linkvisible': {
                'description': _('Set link to have visibility as specified by "visible".'),
                'type': 'string',
            },
            'pose': {
                'type': 'array',
                'items': {
                    'type': 'number',
                }
            },
            'restore': {
                'type': 'boolean',
            },
            'templateinfos': {
                'type': 'object',
            },
            'uri': uriSchema,
            'visible': {
                'type': 'boolean',
            },
        }
    }
}

envclearance = {
    'description': _('Environment clearance in millimeters.'),
    'type': 'number',
}

execute = {
    'description': _('If 1, execute the motion.'),
    'type': 'integer',
}

filteroptions = {
    'description': _('OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked'),
    'type': 'integer',
}

graspsetname = {
    'description': _('Name of the grasp set to use'),
    'type': 'string',
}

quaternion = {
    'description': _('List specifying the quaternion in w,x,y,z format, e.g. [1,0,0,0].'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

robotaccelmult = {
    'description': _('Value in (0,1] defining the percentage of acceleration the robot should move at.'),
    'maximum': 1.0,
    'minimum': 0.0001,
    'type': 'number',
}

robotspeedmult = {
    'description': _('Value in (0,1] defining the percentage of speed the robot should move at.'),
    'maximum': 1.0,
    'minimum': 0.0001,
    'type': 'number',
}

robotname = {
    'description': _('Name of the robot'),
    'type': 'string',
}

robotspeed = {
    'description': _('Value in (0,1] defining the percentage of speed the robot should move at.'),
    'maximum': 1.0,
    'minimum': 0.0001,
    'type': 'number',
}

targetname = {
    'description': _('Name of the target object'),
    'type': 'string',
}

toolname = {
    'description': _('Name of the manipulator. Defaults to currently selected tool'),
    'type': 'string',
}

translation = {
    'description': _('List of x,y,z values of the object in millimeters.'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

transformSchema = {
    'typeName': 'transform',
    'type': 'array',
    'additionalItems': False,
    'maxItems': 7,
    'minItems': 7,
    'prefixItems': [
        {
            'description': _('Rotation w'),
            'type': 'number',
        },
        {
            'description': _('Rotation x'),
            'type': 'number',
        },
        {
            'description': _('Rotation y'),
            'type': 'number',
        },
        {
            'description': _('Rotation z'),
            'type': 'number',
        },
        {
            'description': _('Translation x'),
            'type': 'number',
        },
        {
            'description': _('Translation y'),
            'type': 'number',
        },
        {
            'description': _('Translation z'),
            'type': 'number',
        },
    ],
}

toolPosesSchema = {
    "title": _("Tool poses"),
    "description": _("Tool poses for the robot."),
    "type": "object",
    "properties": {
        "cycleStart": {
            "type": "object",
            "properties": {
                "ikparamname": {
                    "type": "string",
                },
                "toolname": {
                    "type": "string",
                },
                "useSourceContainerIkParams": {
                    "type": "boolean",
                },
            }
        }
    }
}

unit = {
    'default': 'mm',
    'description': _('The unit of the given values.'),
    'type': 'string',
}

Vector3Schema = {
    'additionalItems': False,
    'maxItems': 3,
    'minItems': 3,
    'prefixItems': [
        {
            'description': _('x'),
            'type': 'number',
        },
        {
            'description': _('y'),
            'type': 'number',
        },
        {
            'description': _('z'),
            'type': 'number',
        },
    ],
    'type': 'array',
}

locationCollisionInfos = {
    'description': _('List of external collision IOs to be computed and sent in realtime.'),
    'type': 'array',
    'items': {
        'type': 'object',
        'properties': {
            'containerName': {
                'type': 'string',
            },
            'externalCollisionName': {
                'type': 'string'
            },
            'forceDisableCollisionForPlanning': {
                'type': 'boolean',
            },
            'forceEnableAllLinks': {
                'type': 'boolean',
            },
            'locationName': {
                'type': 'string',
            },
            'setToLastPlaced': {
                'type': 'boolean',
            },
            'useAABB': {
                'type': 'boolean',
            },
        }
    }
}

robotBridgeConnectionInfo = {
    'description': _('Information to set up a client to the robot bridge.'),
    'properties': OrderedDict([
        ('host', {
            'default': '172.0.0.1',
            'description': _('The host address of the robotbridge.'),
            'type': 'string',
        }),
        ('port', {
            'default': 7000,
            'description': _('The port of the robotbridge.'),
            'type': 'integer',
        }),
        ('queueid', {
            'description': _('The requested planning slave id.'),
            'type': 'string',
        }),
        ('use', {
            'description': _('If False, robotbridge is not used.'),
            'type': 'boolean',
        }),
    ]),
    'type': 'object',
}

Internal_SetRobotClientParameters = OrderedDict([
    ('unit', unit),
    ('robotname', robotname),
    ('toolname', toolname),
    ('robotBridgeConnectionInfo', robotBridgeConnectionInfo),
    ('locationCollisionInfos', locationCollisionInfos),
])

Internal_ExecuteCommandParameters = {
    'fireandforget': {
        'default': False,
        'description': _('If True, does not wait for the command to finish and returns immediately. The command remains queued on the server.'),
        'type': 'boolean',
        'x-doNotAddToPayload': True,
    },
    'forcereload': {
        'description': _('If True, then force re-load the scene before executing the task'),
        'isRequired': False,
        'type': 'boolean',
        'x-doNotAddToPayload': True,
    },
    'respawnopts': {
        'description': _('Settings to determine the respawning behavior of planning slaves. Restarts/respawns a planning slave if conditions are met.'),
        'isRequired': False,
        'properties': OrderedDict([
            ('allowrespawn', {
                'default': True,
                'description': _('Allow the planning slave to respawn.'),
                'type': 'boolean',
            }),
            ('forcerespawn', {
                'default': False,
                'description': _('Force the planning slave to respawn.'),
                'type': 'boolean',
            }),
            ('respawnMemoryThreshold', {
                'default': '2*1024*1024*1024',
                'description': _('The amount of memory that the planning slave may occupy before it is respawned.'),
                'type': 'float',
            }),
        ]),
        'type': ['object', 'null'],
        'x-doNotAddToPayload': True,
    },
    'timeout': {
        'default': 10,
        'description': _('Time in seconds after which the command is assumed to have failed.'),
        'type': 'number',
        'x-doNotAddToPayload': True,
    },
}

StandardPlanningServerRequestParameters = [
    {
        'name': 'fnname',
        'description': _('Needs to be "RunCommand"'),
        'schema': {
            'type': 'string',
        }
    },
    {
        'name': 'respawnopts',
        'schema': {
            'properties': OrderedDict([
                ('allowrespawn', {
                    'type': 'boolean',
                }),
                ('forcerespawn', {
                    'type': 'boolean',
                }),
                ('respawnMemoryThreshold', {
                    'description': _('The memory limit before the planning slave is restarted.'),
                    'type': 'integer',
                }),
            ]),
            'type': ['object', 'null'],
        }
    },
    {
        'name': 'slaverequestid',
        'description': _('The id of the requested planning slave'),
        'isRequired': False,
        'schema': {
            'type': 'string',
        }
    },
    {
        'name': 'stamp',
        'description': _('The timestamp of when the command was sent, in seconds.'),
        'schema': {
            'type': 'number',
        }
    },
    {
        'name': 'taskparams',
        'schema': {
            'properties': OrderedDict([
                ('forcereload', {
                    'type': 'boolean',
                }),
                ('tasktype', {
                    'type': 'string',
                }),
                ('sceneparams', {
                    'properties': OrderedDict([
                        ('scenetype', {
                            'description': _("Should be 'mujin'"),
                            'type': 'string',
                        }),
                        ('sceneuri', {
                            'description': _("The Scene's URI."),
                            'type': 'string',
                        }),
                        ('scenefilename', {
                            'description': _("The Scene's file name."),
                            'type': 'string',
                        }),
                        ('scale', MergeDicts(
                            [
                                Vector3Schema,
                                {
                                    'description': _("Values of ['x', 'y', 'z'] correspondingly. Default values are 1.0."),
                                }
                            ],
                            deepcopy=True,
                        )[0]),
                    ]),
                    'type': 'object',
                }),
                ('taskparameters', {
                    'type': 'object',
                    'properties': {
                        'dynamicEnvironmentState': dynamicEnvironmentState,
                    }
                }),
            ]),
            'type': 'object',
        }
    },
    {
        'name': 'userinfo',
        'description': _('An object storing user info such as the locale'),
        'schema': {
            'properties': OrderedDict([
                ('username', {
                    'description': _('Username used for the controller'),
                    'type': 'string',
                }),
                ('locale', {
                    'type': 'string',
                }),
            ]),
            'type': 'object',
        }
    },
]

startItlParameters = {
    # NOTE: This is currently defined in the user config in the key 'itlParameters' and unpacked into the `StartITLProgram` call, but it should be in its own field in the call.
    'description': _('Planning parameters for ITL programs.'),
    'properties': OrderedDict([
        ('allowGrabWithoutTemplateTarget', {
            'default': False,
            'description': _('Deprecated. Only for backwards compatibility.'),  # Only used in test_aurotek.
            'type': 'boolean',
        }),
        ('departOffsetDir', departoffsetdir),
        ('disallowSteppingBackwardAfterGrabRelease', {
            'default': True,
            'description': _('Deprecated. Only for backwards compatibility.'),  # Only used in test_aurotek.
            'type': 'boolean',
        }),
        ('envclearance', envclearance),
        ('planningSmallestObjectSizeForCollision', binpickingparametersschema.planningSmallestObjectSizeForCollisionSchema),
        ('saveRobotFeedbackLog', binpickingparametersschema.saveRobotFeedbackLogSchema),
        ('savetrajectorylog', binpickingparametersschema.savetrajectorylogSchema),
        ('smootherParameters', binpickingparametersschema.smootherParametersSchema),
        ('startline', {  # TODO(felixvd): This does not seem to be implemented.
            'default': 0,
            'description': _('Line of the ITL program to start at. This setting may be ignored by the server.'),
            'type': ['integer', 'null'],
        }),
        ('stepping', {
            'default': False,
            'description': _('If queue mode is "Stepping" (line-by-line execution) or not.'),
            'type': 'boolean',
        }),
        ('toolposes', toolPosesSchema),
    ]),
    'type': 'object',
}
