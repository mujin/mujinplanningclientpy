# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

from . import components


clientType = 'handeyecalibration'

templateArgs = {
    'clientTaskName': 'HandEyeCalibration',
    'extraClassAttributes': 'robot = None  # type: str # type: ignore',
    'extraClientContent': '''\
def ReloadModule(self, **kwargs):
    return self.ExecuteCommand({
        'command': 'ReloadModule',
    }, **kwargs)
''',
    'extraClientStaticFunctions': '',
    'extraConstructorArgs': 'robot,\n',
    'extraConstructorArgsDocstringLines': 'robot (str): Name of the robot, e.g. VP-5243I\n',
    'extraConstructorArgsTypeAnnotation': 'str',
    'extraConstructorContent': 'self.robot = robot\n',
    'extraImports': None,
    'extraSuperConstructorArgs': '',
    'parentClassFile': 'planningclient',
    'parentClassName': 'PlanningClient',
    'tasktype': 'handeyecalibration',
}

services = [
    ('ComputeCalibrationPoses', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': OrderedDict([
                    ('taskparameters', {
                        'properties': OrderedDict([
                            ('primarySensorSelectionInfo', {
                                'paramOrderingIndex': 0,
                            }),
                            ('secondarySensorSelectionInfos', {
                                'paramOrderingIndex': 1,
                            }),
                            ('numsamples', {
                                'paramOrderingIndex': 2,
                            }),
                            ('calibboardvisibility', {
                                'isRequired': True,
                                'paramOrderingIndex': 3,
                            }),
                            ('calibboardLinkName', {
                                'paramOrderingIndex': 4,
                            }),
                            ('calibboardGeomName', {
                                'paramOrderingIndex': 5,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 3000,
                                        'paramOrderingIndex': 6,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('gridindex', {
                                'paramOrderingIndex': 7,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 8,
                            }),
                            ('calibboardObjectName', {
                                'paramOrderingIndex': 9,
                            }),
                            ('minPatternTiltAngle', {
                                'paramOrderingIndex': 10,
                            }),
                            ('maxPatternTiltAngle', {
                                'paramOrderingIndex': 11,
                            }),
                            ('dynamicEnvironmentState', {
                                'paramOrderingIndex': 12,
                            }),
                            ('robot', {
                                'paramOrderingIndex': 13,
                                'fillFromClassMember': 'robot',
                            }),
                            ('kwargs', {}),
                        ]),
                    }),
                ]),
            },
        }],
    }),
    ('SampleCalibrationConfiguration', {
        'parameters': [{
            'name': 'taskparams',
            'schema': {
                'properties': OrderedDict([
                    ('taskparameters', {
                        'properties': OrderedDict([
                            ('primarySensorSelectionInfo', {
                                'paramOrderingIndex': 0,
                            }),
                            ('secondarySensorSelectionInfos', {
                                'paramOrderingIndex': 1,
                            }),
                            ('gridindex', {
                                'paramOrderingIndex': 2,
                            }),
                            ('calibboardvisibility', {
                                'paramOrderingIndex': 3,
                            }),
                            ('calibboardLinkName', {
                                'paramOrderingIndex': 5,
                            }),
                            ('calibboardGeomName', {
                                'paramOrderingIndex': 6,
                            }),
                            ('timeout', MergeDicts(
                                [
                                    components.Internal_ExecuteCommandParameters['timeout'],
                                    {
                                        'default': 3000,
                                        'paramOrderingIndex': 7,
                                    }
                                ],
                                deepcopy=True,
                            )[0]),
                            ('minPatternTiltAngle', {
                                'paramOrderingIndex': 8,
                            }),
                            ('maxPatternTiltAngle', {
                                'paramOrderingIndex': 9,
                            }),
                            ('toolname', {
                                'paramOrderingIndex': 10,
                            }),
                            ('calibboardObjectName', {
                                'paramOrderingIndex': 11,
                            }),
                            ('dynamicEnvironmentState', {
                                'paramOrderingIndex': 12,
                            }),
                            ('robot', {
                                'paramOrderingIndex': 13,
                                'fillFromClassMember': 'robot',
                            }),
                            ('kwargs', {}),
                        ]),
                    }),
                ]),
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
