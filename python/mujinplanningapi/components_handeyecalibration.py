# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from . import _


calibboardGeomName = {
    'type': 'string',
}

calibboardLinkName = {
    'type': 'string',
}

calibboardObjectName = {
    'type': 'string',
}

calibboardvisibility = {
    'type': 'object',
}

gridindex = {
    'description': _('The index of the voxel'),
    'type': 'integer',
}

maxPatternTiltAngle = {
    'description': _('The maximum tilt of the pattern in degrees. Default: 30 degrees'),
    'type': 'number',
}

minPatternTiltAngle = {
    'description': _('The minimum tilt of the pattern in degrees. Default: 10 degrees'),
    'type': 'number',
}

primarySensorSelectionInfo = {
    'description': _('Selects the primary camera that everything will be calibrated against.'),
    'type': 'object',
}

robot = {
    'description': _('The name of the robot (modelName). If not specified - the value used for client initialization will be applied.'),
    'type': 'string',
}

secondarySensorSelectionInfos = {
    'description': _('Selects the secondary camera(s) (assumed to be nearby the primary sensor).'),
    'items': {
        'type': 'object',
    },
    'type': 'array',
}

toolname = {
    'type': 'string',
}
