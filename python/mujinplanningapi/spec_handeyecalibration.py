# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts
from . import UpdateTaskparams

from . import components
from . import components_handeyecalibration
from . import spec_realtimerobot


services = [
    ('ComputeCalibrationPoses', {
        'description': _('Compute a set of calibration poses that satisfy the angle constraints using latin hypercube sampling (or stratified sampling upon failure)'),
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
                                ('primarySensorSelectionInfo', MergeDicts(
                                    [
                                        components_handeyecalibration.primarySensorSelectionInfo,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('secondarySensorSelectionInfos', MergeDicts(
                                    [
                                        components_handeyecalibration.secondarySensorSelectionInfos,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('numsamples', {
                                    'description': _('Number of samples to take. A reasonable number is often between 5 and 25.'),
                                    'isRequired': True,
                                    'type': 'integer',
                                }),
                                ('calibboardvisibility', MergeDicts(
                                    [
                                        components_handeyecalibration.calibboardvisibility,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('calibboardLinkName', components_handeyecalibration.calibboardLinkName),
                                ('calibboardGeomName', components_handeyecalibration.calibboardGeomName),
                                ('gridindex', components_handeyecalibration.gridindex),
                                ('toolname', components_handeyecalibration.toolname),
                                ('calibboardObjectName', components_handeyecalibration.calibboardObjectName),
                                ('minPatternTiltAngle', components_handeyecalibration.minPatternTiltAngle),
                                ('maxPatternTiltAngle', components_handeyecalibration.maxPatternTiltAngle),
                                ('dynamicEnvironmentState', components.dynamicEnvironmentState),
                                ('robot', components_handeyecalibration.robot),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {},
    }),
    ('SampleCalibrationConfiguration', {
        'description': _('Sample a valid calibration pose inside the given voxel and find a corresponding IK solution.'),
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
                                ('primarySensorSelectionInfo', MergeDicts(
                                    [
                                        components_handeyecalibration.primarySensorSelectionInfo,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('secondarySensorSelectionInfos', MergeDicts(
                                    [
                                        components_handeyecalibration.secondarySensorSelectionInfos,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('gridindex', MergeDicts(
                                    [
                                        components_handeyecalibration.gridindex,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('calibboardvisibility', MergeDicts(
                                    [
                                        components_handeyecalibration.calibboardvisibility,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('calibboardLinkName', components_handeyecalibration.calibboardLinkName),
                                ('calibboardGeomName', components_handeyecalibration.calibboardGeomName),
                                ('minPatternTiltAngle', components_handeyecalibration.minPatternTiltAngle),
                                ('maxPatternTiltAngle', components_handeyecalibration.maxPatternTiltAngle),
                                ('toolname', components_handeyecalibration.toolname),
                                ('calibboardObjectName', components_handeyecalibration.calibboardObjectName),
                                ('dynamicEnvironmentState', components.dynamicEnvironmentState),
                                ('robot', components_handeyecalibration.robot),
                            ]),
                        },
                    },
                },
            },
        ),
        'returns': {
            'properties': OrderedDict([
                ('vConfig', {
                    'description': _('The IK solution (joint angles) for the sample.'),
                    'items': {
                        'type': 'number',
                    },
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
]

calibrationSpec = {
    'info': {
        'title': _('Calibration'),
        'description': 'The Calibration API of the Mujin Planning Server.',
        'mujinspecformatversion': '0.0.1',
    },
    'services': MergeDicts(
        [
            OrderedDict(services),
            spec_realtimerobot.realtimeRobotSpec['services'],
        ],
        deepcopy=True
    ),
}
