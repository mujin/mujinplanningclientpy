# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

from . import components


clientType = 'realtimerobot'

templateArgs = {
    'clientTaskName': 'RealtimeRobot',
    'extraClassAttributes': '''\
_robotname = None  # type: str # type: ignore # Optional name of the robot selected
_robotspeed = None  # type: Optional[float] # Speed of the robot, e.g. 0.4
_robotaccelmult = None  # type: Optional[float] # Current robot accel mult
_envclearance = None  # type: float # type: ignore # Environment clearance in millimeters, e.g. 20
_robotBridgeConnectionInfo = None  # type: Optional[str] # dict holding the connection info for the robot bridge.
''',
    'extraClientContent': '',
    'extraClientStaticFunctions': '''\
def GetRobotConnectionInfo(self):
    # type: () -> Optional[str]
    """ """
    return self._robotBridgeConnectionInfo

def SetRobotConnectionInfo(self, robotBridgeConnectionInfo):
    # type: (str) -> None
    """
    Args:
        robotBridgeConnectionInfo:
    """
    self._robotBridgeConnectionInfo = robotBridgeConnectionInfo

def GetRobotName(self):
    # type: () -> Optional[str]
    """ """
    return self._robotname

def SetRobotName(self, robotname):
    # type: (str) -> None
    """
    Args:
        robotname (str):
    """
    self._robotname = robotname

def SetRobotSpeed(self, robotspeed):
    # type: (float) -> None
    """
    Args:
        robotspeed:
    """
    self._robotspeed = robotspeed

def SetRobotAccelMult(self, robotaccelmult):
    # type: (float) -> None
    """
    Args:
        robotaccelmult:
    """
    self._robotaccelmult = robotaccelmult

def ExecuteCommand(self, taskparameters, robotname=None, toolname=None, robotspeed=None, robotaccelmult=None, envclearance=None, timeout=10, fireandforget=False, respawnopts=None, forcereload=False):
    # type: (Dict, Optional[str], Optional[str], Optional[float], Optional[float], Optional[float], float, bool, Any, bool) -> Any
    """Wrapper to ExecuteCommand with robot info specified in taskparameters.

    Executes a command in the task.

    Args:
        taskparameters (dict): Specifies the arguments of the task/command being called.
        robotname (str, optional): Name of the robot
        robotaccelmult (float, optional):
        envclearance (float, optional):
        respawnopts (optional):
        toolname (str, optional): Name of the manipulator. Default: self.toolname
        timeout (float, optional):  (Default: 10)
        fireandforget (bool, optional):  (Default: False)
        robotspeed (float, optional):
        forcereload (bool): If True, then force re-load the scene before executing the task.

    Returns:
        dict: Contains:
            - robottype (str): robot type
            - currentjointvalues (list[float]): current joint values, vector length = DOF
            - elapsedtime (float): elapsed time in seconds
            - numpoints (int): the number of points
            - error (dict): optional error info
            - desc (str): error message
            - type (str): error type
            - errorcode (str): error code
    """
    if robotname is None:
        robotname = self._robotname

    # caller wants to use a different tool
    if toolname is not None:
        # set at the first level
        taskparameters['toolname'] = toolname

    if robotname is not None:
        taskparameters['robotname'] = robotname

    if 'robotspeed' not in taskparameters:
        if robotspeed is None:
            robotspeed = self._robotspeed
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed

    if 'robotaccelmult' not in taskparameters:
        if robotaccelmult is None:
            robotaccelmult = self._robotaccelmult
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult

    if self._robotBridgeConnectionInfo is not None:
        taskparameters['robotBridgeConnectionInfo'] = self._robotBridgeConnectionInfo

    if 'envclearance' not in taskparameters or taskparameters['envclearance'] is None:
        if envclearance is None:
            envclearance = self._envclearance
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance

    return super(RealtimeRobotPlanningClient, self).ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, respawnopts=respawnopts, forcereload=forcereload)
''',
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
    'extraConstructorContent': '''\
self._robotname = robotname
self._robotspeed = robotspeed
self._robotaccelmult = robotaccelmult
self._envclearance = envclearance
self._robotBridgeConnectionInfo = robotBridgeConnectionInfo
''',
    'extraImports': 'from . import json',
    'extraSuperConstructorArgs': '',
    'parentClassFile': 'planningclient',
    'parentClassName': 'PlanningClient',
    'tasktype': 'realtimerobot',
}

services = [
    ('GetJointValues', {
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
                                        'default': 10,
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
    ('MoveToolLinear', {
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
                            ('toolname', {
                                'paramOrderingIndex': 2,
                                'x-doNotAddToPayload': True,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 3,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('robotspeed', {
                                'paramOrderingIndex': 4,
                                'x-doNotAddToPayload': True,
                            }),
                            ('workmaxdeviationangle', {
                                'paramOrderingIndex': 5,
                            }),
                            ('workspeed', {
                                'paramOrderingIndex': 6,
                            }),
                            ('workaccel', {
                                'paramOrderingIndex': 7,
                            }),
                            ('worksteplength', {
                                'paramOrderingIndex': 8,
                            }),
                            ('plannername', {
                                'paramOrderingIndex': 9,
                            }),
                            ('numspeedcandidates', {
                                'paramOrderingIndex': 10,
                            }),
                            ('workminimumcompletetime', {
                                'paramOrderingIndex': 11,
                            }),
                            ('workminimumcompleteratio', {
                                'paramOrderingIndex': 12,
                            }),
                            ('workignorefirstcollisionee', {
                                'paramOrderingIndex': 13,
                            }),
                            ('workignorelastcollisionee', {
                                'paramOrderingIndex': 14,
                            }),
                            ('workignorefirstcollision', {
                                'paramOrderingIndex': 15,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveToHandPosition', {
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
                            ('toolname', {
                                'paramOrderingIndex': 2,
                                'x-doNotAddToPayload': True,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 3,
                                'x-doNotAddToPayload': True,
                            }),
                            ('closegripper', {
                                'isRequired': True,
                                'paramOrderingIndex': 4,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 5,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 6,
                                'x-doNotAddToPayload': True,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('UpdateObjects', {
        'omitFromArguments': ('objectname', 'object_uri'),
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('envstate', {
                                'paramOrderingIndex': 0,
                            }),
                            ('targetname', {
                                'mapsToParamWithCustomAssignment': {
                                    'assignment': "u'mujin:/%s.mujin.dae' % (targetname)",
                                    'param': 'object_uri',
                                },
                                'paramOrderingIndex': 1,
                            }),
                            ('state', {
                                'forceJsonDumps': True,
                                'paramOrderingIndex': 2,
                                'x-specialCase': {
                                    'omitRegularAssignment': True,
                                },
                            }),
                            ('unit', {
                                'paramOrderingIndex': 3,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
        'x-modifiedReturnStatement': "if state is not None:\n    taskparameters['state'] = json.dumps(state)\n",
    }),
    ('Grab', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('Release', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('GetGrabbed', {
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
                                        'default': 10,
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
    ('GetTransform', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('connectedBodyName', {
                                'paramOrderingIndex': 1,
                                'x-assignmentCondition': ' is not None',
                            }),
                            ('linkName', {
                                'paramOrderingIndex': 2,
                                'x-assignmentCondition': ' is not None',
                            }),
                            ('geometryName', {
                                'paramOrderingIndex': 3,
                                'x-assignmentCondition': ' is not None',
                            }),
                            ('geometryPk', {
                                'paramOrderingIndex': 4,
                                'x-assignmentCondition': ' is not None',
                            }),
                            ('unit', {
                                'paramOrderingIndex': 5,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 6,
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
    ('GetLinkParentInfo', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('objectName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('linkName', {
                                'paramOrderingIndex': 1,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('SetTransform', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('translation', {
                                'paramOrderingIndex': 1,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 2,
                            }),
                            ('rotationmat', {
                                'paramOrderingIndex': 3,
                            }),
                            ('quaternion', {
                                'paramOrderingIndex': 4,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 5,
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
        'x-modifiedReturnStatement': "if rotationmat is None and quaternion is None:\n    taskparameters['quaternion'] = [1, 0, 0, 0]\n    log.warn('No rotation is specified. Using identity quaternion.')\n",
    }),
    ('GetOBB', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('linkname', {
                                'paramOrderingIndex': 3,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetInnerEmptyRegionOBB', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('linkname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('GetInstObjectAndSensorInfo', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('instobjectnames', {
                                'paramOrderingIndex': 0,
                            }),
                            ('sensornames', {
                                'paramOrderingIndex': 1,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 3,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('ignoreMissingObjects', {
                                'paramOrderingIndex': 4,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetInstObjectInfoFromURI', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('objecturi', {
                                'customParameterName': 'instobjecturi',
                                'paramOrderingIndex': 0,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('instobjectpose', {
                                'paramOrderingIndex': 3,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetAABB', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('linkname', {
                                'paramOrderingIndex': 3,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('SetLocationTracking', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('cycleIndex', {
                                'paramOrderingIndex': 2,
                            }),
                            ('locationReplaceInfos', {
                                'paramOrderingIndex': 3,
                            }),
                            ('removeLocationNames', {
                                'paramOrderingIndex': 4,
                            }),
                            ('minRobotBridgeTimeStampUS', {
                                'paramOrderingIndex': 5,
                            }),
                            ('dynamicObstacleBaseName', {
                                'paramOrderingIndex': 6,
                            }),
                            ('targetUpdateBaseName', {
                                'paramOrderingIndex': 7,
                            }),
                            ('ioSignalsInfo', {
                                'paramOrderingIndex': 8,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 9,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ResetLocationTracking', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('resetAllLocations', {
                                'paramOrderingIndex': 3,
                            }),
                            ('resetLocationName', {
                                'paramOrderingIndex': 4,
                            }),
                            ('resetLocationNames', {
                                'paramOrderingIndex': 5,
                            }),
                            ('checkIdAndResetLocationName', {
                                'paramOrderingIndex': 6,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
        'returns': {
            'returnPayloadField': 'clearedLocationNames',
        },
        'x-modifiedReturnStatement': "return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)['clearedLocationNames']",
        'x-omitRegularReturnStatement': True,
    }),
    ('GetLocationTrackingInfos', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
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
        'returns': {
            'returnPayloadField': 'activeLocationTrackingInfos',
        },
        'x-modifiedReturnStatement': "return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)['activeLocationTrackingInfos']",
        'x-omitRegularReturnStatement': True,
    }),
    ('UpdateLocationContainerIdType', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('locationName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('containerName', {
                                'paramOrderingIndex': 1,
                            }),
                            ('containerId', {
                                'paramOrderingIndex': 2,
                            }),
                            ('containerType', {
                                'paramOrderingIndex': 3,
                            }),
                            ('trackingCycleIndex', {
                                'paramOrderingIndex': 5,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 6,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
                                        'paramOrderingIndex': 7,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('unit', {
                                'paramOrderingIndex': 8,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ResetLocationTrackingContainerId', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('locationName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('checkContainerId', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
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
    ('RemoveObjectsWithPrefix', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('prefix', {
                                'paramOrderingIndex': 0,
                            }),
                            ('removeNamePrefixes', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
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
                            ('removeLocationNames', {
                                'paramOrderingIndex': 5,
                            }),
                            ('doRemoveOnlyDynamic', {
                                'paramOrderingIndex': 6,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetTrajectoryLog', {
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
                                        'default': 10,
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
                            ('includejointvalues', {
                                'paramOrderingIndex': 3,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ChuckGripper', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('robotname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('grippername', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('toolname', {
                                'paramOrderingIndex': 4,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('UnchuckGripper', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('robotname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('grippername', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('targetname', {
                                'paramOrderingIndex': 4,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 5,
                            }),
                            ('pulloutdist', {
                                'paramOrderingIndex': 6,
                            }),
                            ('deletetarget', {
                                'paramOrderingIndex': 7,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('CalibrateGripper', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('robotname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('grippername', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
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
                            ('toolname', {
                                'paramOrderingIndex': 5,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('StopGripper', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('robotname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('grippername', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
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
                            ('toolname', {
                                'paramOrderingIndex': 5,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveGripper', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('grippervalues', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('grippername', {
                                'default': None,
                                'isRequired': True,
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
                                        'paramOrderingIndex': 5,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('toolname', {
                                'paramOrderingIndex': 6,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('ExecuteRobotProgram', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('robotProgramName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 2,
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
    ('SaveScene', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('filename', {
                                'paramOrderingIndex': 1,
                            }),
                            ('preserveexternalrefs', {
                                'paramOrderingIndex': 2,
                            }),
                            ('externalref', {
                                'paramOrderingIndex': 3,
                            }),
                            ('saveclone', {
                                'paramOrderingIndex': 4,
                            }),
                            ('saveReferenceUriAsHint', {
                                'paramOrderingIndex': 5,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('SaveGripper', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('filename', {
                                'paramOrderingIndex': 2,
                            }),
                            ('manipname', {
                                'paramOrderingIndex': 3,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveJointsToJointConfigurationStates', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('goalJointConfigurationStates', {
                                'customParameterName': 'jointConfigurationStates',
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
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
                                'default': 1,
                                'paramOrderingIndex': 4,
                            }),
                            ('startJointConfigurationStates', {
                                'paramOrderingIndex': 5,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 6,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 7,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('jointStates', {
                                'paramOrderingIndex': 9,
                            }),
                            ('jointindices', {
                                'paramOrderingIndex': 10,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveJoints', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointvalues', {
                                'description': _('List of joint values to move to. Use goaljoints instead.'),
                                'forceCast': 'list',
                                'mapsTo': 'goaljoints',
                                'paramOrderingIndex': 0,
                            }),
                            ('robotJointNames', {
                                'paramOrderingIndex': 1,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 3,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 4,
                                'x-doNotAddToPayload': True,
                            }),
                            ('execute', {
                                'default': 1,
                                'paramOrderingIndex': 5,
                            }),
                            ('startvalues', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 6,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 7,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 8,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('jointindices', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 11,
                            }),
                            ('robotProgramName', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 12,
                            }),
                            ('goaljoints', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 100,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('MoveJointsToPositionConfiguration', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('positionConfigurationName', {
                                'paramOrderingIndex': 0,
                                'x-assignmentCondition': '',
                            }),
                            ('positionConfigurationCandidateNames', {
                                'paramOrderingIndex': 1,
                                'x-assignmentCondition': '',
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('robotspeed', {
                                'paramOrderingIndex': 3,
                                'x-doNotAddToPayload': True,
                            }),
                            ('robotaccelmult', {
                                'paramOrderingIndex': 4,
                                'x-doNotAddToPayload': True,
                            }),
                            ('execute', {
                                'default': 1,
                                'paramOrderingIndex': 5,
                            }),
                            ('startvalues', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 6,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 7,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 8,
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
    ('GetRobotBridgeIOVariables', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('ioname', {
                                'paramOrderingIndex': 0,
                                'x-specialCase': {
                                    'content': "if ioname is not None and len(ioname) > 0:\n    taskparameters['ioname'] = ioname\n",
                                    'omitRegularAssignment': True,
                                },
                            }),
                            ('ionames', {
                                'paramOrderingIndex': 1,
                                'x-specialCase': {
                                    'content': "if ionames is not None and len(ionames) > 0:\n    taskparameters['ionames'] = ionames\n",
                                    'omitRegularAssignment': True,
                                },
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('SetRobotBridgeIOVariables', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('iovalues', {
                                'forceCast': 'list',
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('ComputeIkParamPosition', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('name', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('ComputeIKFromParameters', {
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
                                        'default': 10,
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
    ('ReloadModule', {
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
                                        'default': 10,
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
    ('ShutdownRobotBridge', {
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
                                        'default': 10,
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
    ('GetRobotBridgeState', {
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
                                        'default': 10,
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
    ('ClearRobotBridgeError', {
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
                                        'default': 10,
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
    ('SetRobotBridgePause', {
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
                                        'default': 10,
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
    ('SetRobotBridgeResume', {
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
                                        'default': 10,
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
    ('SetRobotBridgeServoOn', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('isservoon', {
                                'customParameterName': 'servoon',
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 3,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
                                        'paramOrderingIndex': 3,
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
    ('SetRobotBridgeLockMode', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('islockmode', {
                                'paramOrderingIndex': 0,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 3,
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
                                        'paramOrderingIndex': 3,
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
    ('ResetSafetyFault', {
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
                                        'default': 3,
                                        'paramOrderingIndex': 0,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
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
    ('SetRobotBridgeControlMode', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('controlMode', {
                                'paramOrderingIndex': 0,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 3,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
                                        'paramOrderingIndex': 2,
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
    ('GetDynamicObjects', {
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
    ('ComputeRobotConfigsForGraspVisualization', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('targetname', {
                                'paramOrderingIndex': 0,
                            }),
                            ('graspname', {
                                'paramOrderingIndex': 1,
                            }),
                            ('robotname', {
                                'paramOrderingIndex': 2,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 3,
                                'x-doNotAddToPayload': True,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 4,
                                'x-specialCase': {
                                    'content': "if unit is not None:\n    taskparameters['unit'] = unit\n",
                                    'omitRegularAssignment': True,
                                },
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 6,
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
    ('ResetCacheTemplates', {
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
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'default': False,
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
    ('SetRobotBridgeExternalIOPublishing', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('enable', {
                                'forceCast': 'boolean',
                                'paramOrderingIndex': 0,
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
                                        'default': False,
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
    ('RestoreSceneInitialState', {
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
                                        'paramOrderingIndex': 2,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('preserverobotdofvalues', {
                                'default': 1,
                                'paramOrderingIndex': 3,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('RunMotorControlTuningStepTest', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('amplitude', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
        'x-modifiedReturnStatement': "log.warn('sending taskparameters=%r', taskparameters)",
    }),
    ('RunMotorControlTuningMaximulLengthSequence', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('amplitude', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('RunMotorControlTuningDecayingChirp', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('amplitude', {
                                'paramOrderingIndex': 1,
                            }),
                            ('freqMax', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 120,
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
    ('RunMotorControlTuningGaussianImpulse', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('amplitude', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 20,
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
    ('RunMotorControlTuningBangBangResponse', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('amplitude', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 60,
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
    ('RunDynamicsIdentificationTest', {
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
                                        'default': 4,
                                        'isRequired': True,
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
    ('GetTimeToRunDynamicsIdentificationTest', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('jointName', {
                                'paramOrderingIndex': 2,
                            }),
                            ('minJointAngle', {
                                'paramOrderingIndex': 3,
                            }),
                            ('maxJointAngle', {
                                'paramOrderingIndex': 4,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('CalculateTestRangeFromCollision', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('jointName', {
                                'paramOrderingIndex': 2,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 3,
                            }),
                            ('envclearance', {
                                'paramOrderingIndex': 4,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetMotorControlParameterSchema', {
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
                                        'default': 10,
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
    ('GetMotorControlParameter', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('parameterName', {
                                'paramOrderingIndex': 1,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('GetMotorControlParameters', {
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
                                        'default': 10,
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
    ('SetMotorControlParameter', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('jointName', {
                                'paramOrderingIndex': 0,
                            }),
                            ('parameterName', {
                                'paramOrderingIndex': 1,
                            }),
                            ('parameterValue', {
                                'paramOrderingIndex': 2,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('IsProfilingRunning', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
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
    ('StartProfiling', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('clocktype', {
                                'paramOrderingIndex': 0,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
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
    ('StopProfiling', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
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
    ('ReplaceBodies', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('bodieslist', {
                                'paramOrderingIndex': 0,
                                'x-specialCase': {
                                    'content': "taskparameters['replaceInfos'] = bodieslist\n",
                                },
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('replaceInfos', {
                                'paramOrderingIndex': 2,
                            }),
                            ('testLocationName', {
                                'paramOrderingIndex': 3,
                            }),
                            ('testLocationContainerId', {
                                'paramOrderingIndex': 4,
                            }),
                            ('removeNamePrefixes', {
                                'paramOrderingIndex': 5,
                            }),
                            ('removeLocationNames', {
                                'paramOrderingIndex': 6,
                            }),
                            ('doRemoveOnlyDynamic', {
                                'paramOrderingIndex': 7,
                            }),
                            ('unit', {
                                'paramOrderingIndex': 8,
                            }),
                            ('kwargs', {}),
                        ]),
                    },
                },
            },
        }],
    }),
    ('GetState', {
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
                                        'default': 10.0,
                                        'paramOrderingIndex': 0,
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
    ('EnsureSyncWithRobotBridge', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': {
                    'taskparameters': {
                        'properties': OrderedDict([
                            ('syncTimeStampUS', {
                                'paramOrderingIndex': 0,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 10,
                                        'paramOrderingIndex': 1,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('fireandforget', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['fireandforget'],
                                    {
                                        'paramOrderingIndex': 3,
                                        'type': 'boolean',
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
    ('ResetCachedRobotConfigurationState', {
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
                                        'default': 10,
                                        'paramOrderingIndex': 0,
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
]

generatorSettings = {
    'template': ('mujinplanningapi', 'templates/client_template.py.mako'),
    'templateArgs': templateArgs,
    'x-specModifications': {
        'services': OrderedDict(services),
    }
}
