# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

from . import components


clientType = 'realtimeitl3'

templateArgs = {
    'clientTaskName': 'RealtimeITL3',
    'extraClassAttributes': None,
    'extraClientContent': None,
    'extraClientStaticFunctions': None,
    'extraConstructorArgs': '''\
robotname='',
robotspeed=None,
robotaccelmult=None,
envclearance=10.0,
robotBridgeConnectionInfo=None,
''',
    'extraConstructorArgsDocstringLines': '''\
robotname (str, optional): Name of the robot, e.g. VP-5243I
robotspeed (float, optional): Speed of the robot, e.g. 0.4
robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
envclearance (float, optional): Environment clearance in millimeter, e.g. 20
robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
''',
    'extraConstructorArgsTypeAnnotation': 'str, Optional[float], Optional[float], float, Optional[str]',
    'extraConstructorContent': None,
    'extraImports': None,
    'extraSuperConstructorArgs': '''\
robotname=robotname,
robotspeed=robotspeed,
robotaccelmult=robotaccelmult,
envclearance=envclearance,
robotBridgeConnectionInfo=robotBridgeConnectionInfo,
''',
    'parentClassFile': 'realtimerobotplanningclient',
    'parentClassName': 'RealtimeRobotPlanningClient',
    'tasktype': 'realtimeitl3',
}

services = [
    ('SetJointValues', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointvalues', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetITLState', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('robotname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ExecuteTrajectory', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('identifier', {
                                'paramOrderingIndex': 0,
                            }),
                            ('trajectories', {
                                'paramOrderingIndex': 1,
                            }),
                            ('statevalues', {
                                'paramOrderingIndex': 2,
                            }),
                            ('stepping', {
                                'isRequired': True,
                                'paramOrderingIndex': 3,
                            }),
                            ('istep', {
                                'paramOrderingIndex': 4,
                            }),
                            ('cycles', {
                                'paramOrderingIndex': 5,
                            }),
                            ('restorevalues', {
                                'paramOrderingIndex': 6,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 7,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 8,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 9,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 10,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 11,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ExecuteTrajectoryStep', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('reverse', {
                                'paramOrderingIndex': 0,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 1,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 2,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 3,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 4,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 5,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                        ]),
                    },
                },
            },
        }],
    }),
    ('PauseExecuteTrajectory', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ResumeExecuteTrajectory', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ComputeRobotConfigsForCommandVisualization', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('executiongraph', {
                                'paramOrderingIndex': 0,
                            }),
                            ('commandindex', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 2,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 3,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ComputeRobotJointValuesForCommandVisualization', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('program', {
                                'paramOrderingIndex': 0,
                            }),
                            ('commandindex', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 2,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 3,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('PlotProgramWaypoints', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 1,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': True,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('StartITLProgram', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('programName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 1,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 3,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 4,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('StopITLProgram', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GenerateExecutionGraph', {
        'description': _('Generates a list of commands for the ITL program.'),
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('programName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('commandTimeout', {
                                'paramOrderingIndex': 1,
                            }),
                            ('totalTimeout', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 3,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 4,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('PopulateTargetInContainer', {
        'description': _('Populates targets in the container using populateFn.'),
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('locationName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('populateTargetUri', {
                                'paramOrderingIndex': 1,
                            }),
                            ('populateFnName', {
                                'paramOrderingIndex': 2,
                            }),
                            ('containerMetaData', {
                                'paramOrderingIndex': 3,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 20,
                                        'paramOrderingIndex': 4,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
]

generatorSettings = {
    'template': ('mujinplanningapi', 'templates/client_template.py.mako'),
    'templateArgs': templateArgs,
    'x-specModifications': {
        'services': OrderedDict(services),
    }
}
