# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

from . import components


clientType = 'binpicking'

templateArgs = {
    'clientTaskName': 'Binpicking',
    'extraClassAttributes': 'regionname = None  # type: Optional[str] # The default region for Pick/Place calls\n',
    'extraClientContent': None,
    'extraClientStaticFunctions': None,
    'extraConstructorArgs': '''\
regionname=None,
robotname='',
robotspeed=None,
robotaccelmult=None,
envclearance=10.0,
robotBridgeConnectionInfo=None,
targetname=None,
toolname=None,
''',
    'extraConstructorArgsDocstringLines': '''\
regionname (str, optional): Name of the bin, e.g. container1
robotname (str, optional): Name of the robot, e.g. VP-5243I
robotspeed (float, optional): Speed of the robot, e.g. 0.4
robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
envclearance (str, optional): Environment clearance in millimeter, e.g. 20
robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
targetname (str, optional): Name of the target, e.g. plasticnut-center
toolname (str, optional): Name of the manipulator, e.g. 2BaseZ
''',
    'extraConstructorArgsTypeAnnotation': 'Optional[str], str, Optional[float], Optional[float], float, Optional[str], Optional[str], Optional[str]',
    'extraConstructorContent': 'self.regionname = regionname\n',
    'extraImports': None,
    'extraSuperConstructorArgs': '''\
robotname=robotname,
robotspeed=robotspeed,
robotaccelmult=robotaccelmult,
envclearance=envclearance,
robotBridgeConnectionInfo=robotBridgeConnectionInfo,
targetname=targetname,
toolname=toolname,
''',
    'parentClassFile': 'realtimerobotplanningclient',
    'parentClassName': 'RealtimeRobotPlanningClient',
    'tasktype': 'binpicking',
}

services = [
    ('PickAndPlace', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('goaltype', {
                                'paramOrderingIndex': 0,
                            }),
                            ('goals', {
                                'paramOrderingIndex': 1,
                            }),
                            ('targetnamepattern', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 2,
                            }),
                            ('approachoffset', {
                                'default': 30,
                                'paramOrderingIndex': 3,
                            }),
                            ('departoffsetdir', {
                                'default': [
                                    0,
                                    0,
                                    50,
                                ],
                                'paramOrderingIndex': 4,
                            }),
                            ('destdepartoffsetdir', {
                                'default': [
                                    0,
                                    0,
                                    30,
                                ],
                                'paramOrderingIndex': 5,
                            }),
                            ('deletetarget', {
                                'default': 0,
                                'paramOrderingIndex': 6,
                            }),
                            ('debuglevel', {
                                'default': 4,
                                'paramOrderingIndex': 7,
                            }),
                            ('movetodestination', {
                                'default': 1,
                                'paramOrderingIndex': 8,
                            }),
                            ('freeinc', {
                                'default': [
                                    0.08,
                                ],
                                'paramOrderingIndex': 9,
                            }),
                            ('worksteplength', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 10,
                            }),
                            ('densowavearmgroup', {
                                'default': 5,
                                'paramOrderingIndex': 11,
                            }),
                            ('regionname', {
                                'paramOrderingIndex': 12,
                            }),
                            ('cameranames', {
                                'paramOrderingIndex': 13,
                            }),
                            ('envclearance', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 14,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 15,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 16,
                                'x-doNotAddToPayload': True,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 1000,
                                        'paramOrderingIndex': 17,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('leaveoffsetintool', {
                                'paramOrderingIndex': 18,
                            }),
                            ('desttargetname', {
                                'paramOrderingIndex': 19,
                            }),
                            ('destikparamnames', {
                                'paramOrderingIndex': 20,
                            }),
                            ('graspsetname', {
                                'paramOrderingIndex': 21,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
        'x-methodStartSetup': '''\
if worksteplength is None:
    worksteplength = 0.01
assert (targetnamepattern is not None)
if regionname is None:
    regionname = self.regionname
''',
    }),
    ('StartPickAndPlaceThread', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('goaltype', {
                                'paramOrderingIndex': 0,
                                'x-specialCase': {
                                    'content': "if goals is not None:\n    taskparameters['goaltype'] = goaltype\n",
                                    'omitRegularAssignment': True,
                                },
                            }),
                            ('goals', {
                                'mapsTo': 'orderedgoals',
                                'paramOrderingIndex': 1,
                            }),
                            ('targetnamepattern', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 2,
                            }),
                            ('approachoffset', {
                                'default': 30,
                                'paramOrderingIndex': 3,
                            }),
                            ('departoffsetdir', {
                                'default': [
                                    0,
                                    0,
                                    50,
                                ],
                                'paramOrderingIndex': 4,
                            }),
                            ('destdepartoffsetdir', {
                                'default': [
                                    0,
                                    0,
                                    30,
                                ],
                                'paramOrderingIndex': 5,
                            }),
                            ('deletetarget', {
                                'default': 0,
                                'paramOrderingIndex': 6,
                            }),
                            ('debuglevel', {
                                'default': 4,
                                'paramOrderingIndex': 7,
                            }),
                            ('movetodestination', {
                                'default': 1,
                                'paramOrderingIndex': 8,
                            }),
                            ('worksteplength', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 9,
                            }),
                            ('regionname', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 10,
                            }),
                            ('envclearance', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 11,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 12,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 13,
                                'x-doNotAddToPayload': True,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 14,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('densowavearmgroup', {
                                'default': 5,
                                'paramOrderingIndex': 15,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
        'x-methodStartSetup': '''\
if worksteplength is None:
    worksteplength = 0.01
assert (targetnamepattern is not None)
if regionname is None:
    regionname = self.regionname
''',
    }),
    ('StopPickPlaceThread', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('resetExecutionState', {
                                'default': True,
                                'paramOrderingIndex': 0,
                            }),
                            ('resetStatusPickPlace', {
                                'default': False,
                                'isRequired': True,
                                'paramOrderingIndex': 1,
                            }),
                            ('finishCode', {
                                'default': None,
                                'isRequired': True,
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
                                        'default': False,
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
    ('GetPickPlaceStatus', {
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ComputeIK', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('toolname', {
                                'paramOrderingIndex': 0,
                                'x-doNotAddToPayload': True,
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
                            ('iktype', {
                                'paramOrderingIndex': 2,
                            }),
                            ('quaternion', {
                                'paramOrderingIndex': 3,
                            }),
                            ('translation', {
                                'paramOrderingIndex': 4,
                            }),
                            ('direction', {
                                'paramOrderingIndex': 5,
                            }),
                            ('angle', {
                                'paramOrderingIndex': 6,
                            }),
                            ('freeincvalue', {
                                'paramOrderingIndex': 7,
                            }),
                            ('filteroptions', {
                                'paramOrderingIndex': 8,
                            }),
                            ('limit', {
                                'paramOrderingIndex': 9,
                            }),
                            ('preshape', {
                                'paramOrderingIndex': 10,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('InitializePartsWithPhysics', {
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
                            ('targeturi', {
                                'paramOrderingIndex': 1,
                            }),
                            ('numtargets', {
                                'paramOrderingIndex': 2,
                            }),
                            ('regionname', {
                                'paramOrderingIndex': 3,
                            }),
                            ('duration', {
                                'paramOrderingIndex': 4,
                            }),
                            ('basename', {
                                'paramOrderingIndex': 5,
                            }),
                            ('deleteprevious', {
                                'paramOrderingIndex': 6,
                            }),
                            ('forcegravity', {
                                'paramOrderingIndex': 7,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
        'x-modifiedReturnStatement': "if 'containername' not in taskparameters:\n    taskparameters['containername'] = self.regionname\n",
    }),
    ('StopPhysicsThread', {
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
                                        'paramOrderingIndex': 99,
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
    ('JitterPartUntilValidGrasp', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('toolname', {
                                'paramOrderingIndex': 0,
                                'x-doNotAddToPayload': True,
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
                            ('targetname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('graspsetname', {
                                'paramOrderingIndex': 3,
                            }),
                            ('approachoffset', {
                                'paramOrderingIndex': 4,
                            }),
                            ('departoffsetdir', {
                                'paramOrderingIndex': 5,
                            }),
                            ('destdepartoffsetdir', {
                                'paramOrderingIndex': 6,
                            }),
                            ('leaveoffsetintool', {
                                'paramOrderingIndex': 7,
                            }),
                            ('desttargetname', {
                                'paramOrderingIndex': 8,
                            }),
                            ('destikparamnames', {
                                'paramOrderingIndex': 9,
                            }),
                            ('jitterangle', {
                                'paramOrderingIndex': 10,
                            }),
                            ('jitteriters', {
                                'paramOrderingIndex': 11,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveToDropOff', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('dropOffInfo', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 2,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 3,
                                'x-doNotAddToPayload': True,
                            }),
                            ('execute', {
                                'paramOrderingIndex': 4,
                            }),
                            ('startvalues', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 5,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 6,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 7,
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
    ('IsRobotOccludingBody', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('bodyname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('cameraname', {
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
    ('GetPickedPositions', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('unit', {
                                'default': 'm',
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetPickAndPlaceLog', {
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
                            ('startindex', {
                                'paramOrderingIndex': 1,
                            }),
                            ('num', {
                                'paramOrderingIndex': 2,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveRobotOutOfCameraOcclusion', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('regionname', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 0,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 1,
                                'x-doNotAddToPayload': True,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 2,
                                'x-doNotAddToPayload': True,
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
                            ('cameranames', {
                                'paramOrderingIndex': 4,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
        'x-methodStartSetup': 'if regionname is None:\n    regionname = self.regionname\n',
    }),
    ('PausePickPlace', {
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ResumePickPlace', {
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('SendStateTrigger', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('stateTrigger', {
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
    ('GetBinpickingState', {
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
    ('SetStopPickPlaceAfterExecutionCycle', {
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('PutPartsBack', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('trajectory', {
                                'customParameterName': 'trajectoryxml',
                                'paramOrderingIndex': 0,
                            }),
                            ('numparts', {
                                'paramOrderingIndex': 1,
                            }),
                            ('toolname', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 2,
                            }),
                            ('grippervalues', {
                                'paramOrderingIndex': 3,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 100,
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
    ('GenerateGraspModelFromIkParams', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('graspsetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('targeturi', {
                                'paramOrderingIndex': 1,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 3,
                                'x-doNotAddToPayload': True,
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
        'x-modifiedReturnStatement': 'return self.ExecuteCommand(taskparameters, robotname=robotname, toolname=toolname, timeout=timeout)',
        'x-omitRegularReturnStatement': True,
    }),
    ('CheckGraspModelIk', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('graspsetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('targeturi', {
                                'paramOrderingIndex': 1,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('ikparamnames', {
                                'default': None,
                                'isRequired': True,
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('SetCurrentLayoutDataFromPLC', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('containername', {
                                'paramOrderingIndex': 0,
                            }),
                            ('containerLayoutSize', {
                                'paramOrderingIndex': 1,
                            }),
                            ('destObstacleName', {
                                'paramOrderingIndex': 2,
                            }),
                            ('ioVariableName', {
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
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ClearVisualization', {
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
    ('GetPlanStatistics', {
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
    ('SetCurrentLayoutDataSendOnObjectUpdateData', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('doUpdate', {
                                'paramOrderingIndex': 0,
                            }),
                            ('containername', {
                                'paramOrderingIndex': 1,
                            }),
                            ('containerLayoutSize', {
                                'paramOrderingIndex': 2,
                            }),
                            ('ioVariableName', {
                                'paramOrderingIndex': 3,
                            }),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': True,
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
    ('StartPackFormationComputationThread', {
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
                            ('debuglevel', {
                                'default': 4,
                                'paramOrderingIndex': 1,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 2,
                                'x-doNotAddToPayload': True,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('StopPackFormationComputationThread', {
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
    ('VisualizePackingState', {
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
    ('VisualizePackFormationResult', {
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
                            ('initializeCameraPosition', {
                                'paramOrderingIndex': 2,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetPackFormationSolution', {
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
    ('GetPackItemPoseInWorld', {
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
    ('ManuallyPlacePackItem', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('packFormationComputationResult', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 0,
                            }),
                            ('inputPartIndex', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 1,
                            }),
                            ('placeLocationNames', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 2,
                            }),
                            ('placedTargetPrefix', {
                                'paramOrderingIndex': 3,
                                'x-assignmentCondition': '',
                            }),
                            ('dynamicGoalsGeneratorParameters', {
                                'paramOrderingIndex': 4,
                            }),
                            ('orderNumber', {
                                'paramOrderingIndex': 5,
                            }),
                            ('numLeftToPick', {
                                'paramOrderingIndex': 6,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'paramOrderingIndex': 7,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 8,
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
    ('SendPackFormationComputationResult', {
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
    ('GetLatestPackFormationResultList', {
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
    ('ClearPackingStateVisualization', {
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
    ('ValidatePackFormationResultList', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('packFormationResultList', {
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
    ('ComputeSamePartPackResultBySimulation', {
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
                                        'default': 100,
                                        'paramOrderingIndex': 0,
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
    ('HasDetectionObstacles', {
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
                                        'default': 100,
                                        'paramOrderingIndex': 0,
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
