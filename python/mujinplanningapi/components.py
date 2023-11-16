# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts


debuglevel = {
    'description': _('Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE.'),
    'type': 'integer',
}

dynamicEnvironmentState = {
    'description': _('The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.'),
    'items': {
        'type': 'object',
    },
    'type': 'array',
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
        'type': 'object',
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
            'type': 'object',
        }
    },
    {
        'name': 'slaverequestid',
        'description': _('The id of the requested planning slave'),
        'isRequired': False,
        'schema': {
            'type': 'integer',
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
