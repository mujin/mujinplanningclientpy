# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts
from . import UpdateTaskparams

from . import components
from . import components_realtimerobot
from . import spec_realtimerobot
from mujinbinpickingmanager.schema import binpickingparametersschema  # TODO(felixvd): Move into this repository
from mujinteachworkermanager.itlprogramparameterschema import itlProgramParameterSchema

services = [
    ('SetJointValues', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('jointvalues', MergeDicts(
                                    [
                                        components_realtimerobot.jointvalues,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('robotname', components.robotname),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('GetITLState', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('robotname', components.robotname),
                                ('robotBridgeConnectionInfo', components.robotBridgeConnectionInfo),
                                ('locationCollisionInfos', components.locationCollisionInfos),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('ExecuteTrajectory', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('identifier', {
                                    'isRequired': True,
                                }),
                                ('trajectories', {
                                    'isRequired': True,
                                }),
                                ('statevalues', {
                                    'type': 'array'
                                }),
                                ('stepping', {
                                    'default': False,
                                }),
                                ('istep', {
                                    'type': 'boolean',
                                }),
                                ('cycles', {
                                    'default': 1,
                                }),
                                ('restorevalues', {
                                    'type': 'array',
                                }),
                                ('envclearance', MergeDicts(
                                    [
                                        components.envclearance,
                                        {
                                            'default': 15,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('robotspeed', components.robotspeed),
                                ('robotaccelmult', components.robotaccelmult),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('ExecuteTrajectoryStep', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('reverse', {
                                    'default': False,
                                    'isRequired': True,
                                    'type': 'boolean',
                                }),
                                ('envclearance', MergeDicts(
                                    [
                                        components.envclearance,
                                        {
                                            'default': 15,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('robotspeed', components.robotspeed),
                                ('robotaccelmult', components.robotaccelmult),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('PauseExecuteTrajectory', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
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
        ),
        'returns': {},
    }),
    ('ResumeExecuteTrajectory', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
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
        ),
        'returns': {},
    }),
    ('ComputeRobotConfigsForCommandVisualization', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('executiongraph', {
                                    'isRequired': True,
                                }),
                                ('commandindex', {
                                    'default': 0,
                                }),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('ComputeRobotJointValuesForCommandVisualization', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('program', {
                                    'isRequired': True,
                                }),
                                ('commandindex', {
                                    'default': 0,
                                }),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('PlotProgramWaypoints', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
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
        ),
        'returns': {},
    }),
    ('StartITLProgram', {
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': MergeDicts(
                                [
                                    components.Internal_SetRobotClientParameters,
                                    OrderedDict([
                                        ('programName', {
                                            'isRequired': True,
                                            'type': 'string',
                                        }),
                                        ('debuglevel', components.debuglevel),
                                        ('defaultItlProgramParams', itlProgramParameterSchema),
                                        ('departOffsetDir', components.departoffsetdir),
                                        ('envclearance', components.envclearance),
                                        ('executionid', {
                                            'type': 'string',
                                        }),
                                        ('itlCacheMode', {
                                            'type': 'string',
                                        }),
                                        ('parameters', {
                                            'type': 'object',
                                            'additionalProperties': True,
                                        }),
                                        ('planningSmallestObjectSizeForCollision', binpickingparametersschema.planningSmallestObjectSizeForCollisionSchema),
                                        ('programCommit', {
                                            'type': ['string', 'null'],
                                        }),
                                        ('restorescene', {
                                            'type': 'boolean',
                                        }),
                                        ('robotspeed', components.robotspeed),
                                        ('robotaccelmult', components.robotaccelmult),
                                        ('saveRobotFeedbackLog', binpickingparametersschema.saveRobotFeedbackLogSchema),
                                        ('savetrajectorylog', binpickingparametersschema.savetrajectorylogSchema),
                                        ('smootherParameters', binpickingparametersschema.smootherParametersSchema),
                                        ('stamp', {
                                            'type': 'number'
                                        }),
                                    ]),
                                ],
                                deepcopy=True,
                            )[0],
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('StopITLProgram', {
        'description': _('Stops the ITL program'),
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
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
        ),
        'returns': {},
    }),
    ('GenerateExecutionGraph', {
        'description': _('Generates a list of commands for the ITL program.'),
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('programName', {
                                    'isRequired': True,
                                }),
                                ('commandTimeout', {
                                    'default': 0.2,
                                }),
                                ('totalTimeout', {
                                    'default': 1.0,
                                }),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('PopulateTargetInContainer', {
        'description': _('Populates targets in container using populateFn.'),
        'parameters': UpdateTaskparams(
            components.StandardPlanningServerRequestParameters,
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('locationName', {
                                    'isRequired': True,
                                }),
                                ('populateTargetUri', {
                                    'isRequired': True,
                                }),
                                ('populateFnName', {
                                    'isRequired': True,
                                }),
                                ('containerMetaData', {
                                    'type': 'object',
                                }),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
]

realtimeITL3Spec = {
    'info': {
        'title': _('RealtimeITL3'),
        'description': 'The ITL Planning (v3) API of the Mujin Planning Server.',
        'mujinspecformatversion': '0.0.1',
    },
    'services': MergeDicts(
        [
            OrderedDict(services),
            spec_realtimerobot.realtimeRobotSpec['services'],
        ],
        deepcopy=True
    )[0],
}
