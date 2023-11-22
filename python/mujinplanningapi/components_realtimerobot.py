# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

import sys
if sys.version_info[0] >= 3:
    raise Exception("This file cannot be used until binpickingui is ported to python3, or we put the binpicking schemas in a separate module")

from . import components
from mujinbinpickingmanager.schema import binpickingparametersschema


amplitude = {
    'description': _('The amplitude.'),
    'type': 'number',
}

doRemoveOnlyDynamic = {
    'description': _('If True, removes objects that were added through dynamic means such as UpdateObjects/UpdateEnvironmentState. Default: False'),
    'type': 'boolean',
}

executetimeout = {
    'default': 1.0,
    'description': _('Time in seconds after which the command is assumed to have failed.'),
    'type': 'number',
}

freqMax = {
    'description': _('The maximum frequency in Hz'),
    'type': 'number',
}

goals = {
    'description': _('Flat list of goals, e.g. two 5D ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

goaltype = {
    'description': _('Type of the goal, e.g. translationdirection5d'),
    'type': 'string',
}

grippername = {
    'description': _('Name of the gripper.'),
    'type': 'string',
}

ikparamnames = {
    'description': _('If graspset does not exist, use the ikparamnames to initialize the grasp.'),
    'items': {
        'type': 'string',
    },
    'type': 'array',
}

ioSignalsInfo = {
    'description': _('Struct for dictating if any IO signals should be written on receiving detection results'),
    'type': 'object',
}

ioname = {
    'description': _('One IO name to read/write'),
    'type': 'string',
}

ionames = {
    'description': _('A list of IO names to read/write'),
    'type': 'array',
}

jointName = {
    'description': _('The name of the joint.'),
    'type': 'string',
}

jointindices = {
    'description': _('List of corresponding joint indices, default is range(len(jointvalues))'),
    'items': {
        'type': 'integer',
    },
    'type': 'array',
}

jointvalues = {
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

linknameAABB = {
    'description': _('Name of link to use for the AABB. If not specified, uses entire target.'),
    'type': 'string',
}

linknameOBB = {
    'description': _('Name of link to use for OBB. If not specified, uses entire target.'),
    'type': 'string',
}

locationCollisionInfos = {
    'description': _('List of external collision IOs to be computed and sent in realtime.'),
    'type': 'object',
}

minimumgoalpaths = {
    'description': _('Number of solutions the planner must provide before it is allowed to finish.'),
    'type': 'integer',
}

moveStraightParams = MergeDicts(
    [
        binpickingparametersschema.moveStraightParamsSchema,
        {
            'description': _('A set of parameters defining how the robot behaves during linear motions.'),
        }
    ],
    deepcopy=True,
)[0]

removeLocationNames = {
    'items': {
        'type': 'string',
    },
    'type': 'array',
}

removeNamePrefixes = {
    'description': _('Names of prefixes to match with when removing items'),
    'items': {
        'type': 'string',
    },
    'type': 'array',
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

rotationmat = {
    'description': _('List specifying the rotation matrix in row major format, e.g. [1,0,0,0,1,0,0,0,1]'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

startvalues = {
    'description': _('The robot joint values to start the motion from.'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

workaccel = {
    'description': _('[angleaccel, transaccel] in deg/s^2 and mm/s^2'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

workspeed = {
    'description': _('[anglespeed, transspeed] in deg/s and mm/s'),
    'items': {
        'type': 'number',
    },
    'type': 'array',
}

forceTorqueBasedEstimatorParameters = OrderedDict([
    ('description', _('A set of parameters for force-torque based estimation.')),
    ('type', 'object'),
    ('properties', OrderedDict([
        ('use', {
            'description': _('If False, forceTorqueBasedEstimatorParameters are not used.'),
            'type': 'boolean',
        }),
        ('handCenterOfMass', {
            'description': _('The location of the center of mass'),
            'items': {
                'type': 'number',
            },
            'type': 'array',
        }),
        ('objectCenterOfMass', {
            'description': _('The location of the center of mass'),
            'items': {
                'type': 'number',
            },
            'type': 'array',
        }),
        ('robotForceTorqueCalibrationPositionNames', {
            'description': _('Names of robot positions that are suitable for calibrating the force sensor.'),
            'items': {
                'type': 'string',
            },
            'type': 'array',
        }),
        ('robotForceTorqueCalibrationPositionName', {
            'description': _('Name of a robot position that is suitable for calibrating the force sensor.'),
            'type': 'string',
        }),
    ])),
])

jittererParameters = OrderedDict([
    ('maxJitter', {
        'description': _('Maximum amount of jitter applied to a joint during one iteration.'),
        'type': 'number',
    }),
    ('maxJitterIterations', {
        'description': _('Maximum number of iterations to use during jittering.'),
        'type': 'number',
    }),
    ('maxJitterLinkDist', {
        'description': _('mm.'),
        'type': 'number',
    }),
    ('jitterBiasDirection', {
        'description': _('mm. Not normalized'),
        'items': {
            'type': 'number',
        },
        'type': 'array',
    }),
    ('jitterNeighDistThresh', {
        'type': 'number',
    }),
    ('useWorkspaceJitterer', {
        'type': 'boolean',
    }),
])

pathPlannerParameters = OrderedDict([
    ('pathPlannerName', {
        'description': _('The name of the path planner. Needs to be one of the following: birrt, BasicRRT, DualSpaceTreeSearch,  (Default: birrt)'),
        'type': 'string',
    }),
    ('dynamicsConstraintsType', {
        'description': _('If given, has to be a string. Possible values: IgnoreTorque, NominalTorque, InstantaneousTorque, Unknown. (reference: ConvertDynamicsConstraints function in controllercommon)'),
        'type': 'string',
    }),
    ('maxiter', {
        'description': _('Maximum number of path planning iterations. (Default: 4000)'),
        'type': 'string',
    }),
    ('maxPlanningTime', {
        'description': _('Maximum amount of time to spend in path planning.'),
        'type': 'string',
    }),
    ('steplength', {
        'description': _('Maximum step length of the resulting trajectory.'),
        'type': 'string',
    }),
])

moveJointsParameters = OrderedDict([
    ('unit', components.unit),
    ('constraintToolDirection', {
        'items': {
            'type': 'number',
        },
        'type': 'array',
    }),
    ('departOffsetDir', {
        'description': _('Direction in which to apply the offset when departing from the pick/place operation.'),
        'items': {
            'type': 'number',
        },
        'type': 'array',
    }),
    ('departMinimumCompleteRatio', {
        'description': _('The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0.'),
        'type': 'number',
    }),
    ('departOffsetAwayFromGravity', {
        'description': _('The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir.'),
        'type': 'number',
    }),
    ('trajname', {
        'type': 'string',
    }),
    ('disablebodies', {
        'type': 'boolean',
    }),
    ('ignoreGrabbingTarget', {
        'type': 'boolean',
    }),
    ('jointthresh', {
        'type': 'number',
    }),
    ('envclearance', components.envclearance),
    ('jitter', {
        'type': 'number',
    }),
    ('execute', components.execute),
    ('executionFilterFactor', {
        'type': 'number',
    }),
    ('filtertraj', {
        'type': 'boolean',
    }),
    ('locationCollisionInfos', locationCollisionInfos),
    ('currentlimitratios', {
        'description': _("The joints' current limit ratios."),
        'items': {
            'type': 'number',
        },
        'type': 'array',
    }),
    ('goalJointThreshold', {
        'description': _('Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time.'),
        'items': {
            'type': 'number',
        },
        'type': 'array',
    }),
    ('goalWorkspaceThreshold', {
        'description': _('Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful.'),
        'type': 'number',
    }),
    ('calibrategripper', {
        'type': 'boolean',
    }),
    ('departAccel', {
        'type': 'number',
    }),
    ('maxManipAccel', {
        'type': 'number',
    }),
    ('maxJitterLinkDist', jittererParameters['maxJitterLinkDist']),
    ('pathPlannerParameters', pathPlannerParameters),
    ('moveStraightParams', moveStraightParams),
    ('forceTorqueBasedEstimatorParameters', forceTorqueBasedEstimatorParameters),
    ('dynamicEnvironmentState', components.dynamicEnvironmentState),
    ('savetrajectorylog', {
        'description': _('If True, will save the commanded (input) trajectories before they are executed'),
        'type': 'boolean',
    }),
    ('saveRobotFeedbackLog', {
        'description': _('If True, will tell robotbridge to save trajectory files'),
        'type': 'boolean',
    }),
    ('loadRobotFeedbackLog', {
        'description': _('If True, will tell robotbridge to load the robot feedback log after trajectory ends'),
        'type': 'boolean',
    }),
    ('saveConcatenateTrajectoryLog', {
        'description': _('If True, will save trajectories used for inputs of concatenate trajectory functions'),
        'type': 'boolean',
    }),
    ('saveFilterTrajectoryLog', {
        'description': _('If True, will save trajectories used for filtering, such as SmartFilter'),
        'type': 'boolean',
    }),
    ('executionConnectingTrajReverseMult', {
        'description': _('Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally.'),
        'type': 'number',
    }),
    ('executionReverseRecoveryDistance', {
        'description': _('Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm).'),
        'type': 'number',
    }),
    ('debuglevel', components.debuglevel),
    ('jittererParameters', {
        'type': 'object',
        'properties': jittererParameters,
    }),
    ('gripperInfo', {
        'description': _('TODO(felixvd): Check if this really propagates.'),
        'type': 'object',
    }),
])

smootherParameters = OrderedDict([
    ('smootherplannername', {
        'description': _('Name of the smoother (planner) (Default: parabolicsmoother2)'),
        'type': 'string',
    }),
    ('smootheriterations', {
        'description': _('Maximum number of iterations for the smoother (Default: 3000)'),
        'type': 'string',
    }),
])

CanAcceptUpdateOnLocationParameters = OrderedDict([
    ('locationName', {
        'description': _('Name of the location to update.'),
        'type': 'string',
    }),
    ('locationContainerId', {}),
    ('imageStartTimeStampMS', {
        'type': 'integer',
    }),
    ('callerid', {
        'description': _('The name of the caller (only used internally)'),
        'type': 'string',
    }),
])

Internal_MoveCommandDecoratorParameters = OrderedDict([
    ('robotspeed', components.robotspeed),
    ('speed', {
        'deprecated': True,
        'description': _('Use robotspeed instead.'),
    }),
    ('robotaccelmult', components.robotaccelmult),
    ('ionames', ionames),
])

Internal_MoveToParameters = OrderedDict([
    ('goaltype', goaltype),
    ('goals', goals),
    ('instobjectname', {
        'description': _('If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position.'),
        'type': 'string',
    }),
    ('ikparamname', {
        'description': _('If goaltype is not set and both instobjectname and ikparamname are set, use ikparamname of instobjectname as target position.'),
        'type': 'string',
    }),
    ('ikparamoffset', {
        'items': {
            'type': 'number',
        },
        'type': 'array',
    }),
])

Internal_RealtimeRobotTaskInitParameters = OrderedDict([
    ('setPlanningModels', {
        'default': True,
        'description': _("If True, then call setPlanningModels which will set the robot's weights and resolutions automatically. If False, then do not touch the robot values. (Default: True)"),
        'type': 'boolean',
    }),
    ('doprofiling', {
        'description': _('If True, then gather profiling data'),
        'type': 'boolean',
    }),
    ('zmqctx', {
        'description': _('ZMQ Context'),
        'type': 'object',
    }),
    ('skipTrajectoryExecution', {
        'description': _('If True, actual execution of the trajectory is skipped. This can speed up simulation with viewer disabled.'),
        'type': 'boolean',
    }),
    ('logdir', {
        'description': _('Directory to save the logs (Default: OpenRAVE home directory)'),
        'type': 'string',
    }),
    ('trajectoriessavedir', {
        'description': _('Directory to save the trajectories'),
        'type': 'string',
    }),
    ('locationManagerId', {
        'description': _("A string handed to 'SyncEnvironmentFromOriginal' as part of 'noSyncBodyPrefixes', to avoid bodies clashing with the moving robot while environment is being synced. Used to identify objects that are used for planning"),
        'type': 'string',
    }),
    ('useJointLogMode', {
        'description': _('If 1 then will use joint log in the same environment. If 2 will create a separate environment and use joint log streaming. If 0, then do not create the joint log modules. (Default: 1)'),
        'type': 'integer',
    }),
    ('capturelog', {
        'description': _('If True, will read very accurate values from the robot control log (this is slow)'),
        'type': 'boolean',
    }),
    ('savetrajectorylog', {
        'description': _('If True, will save the commanded (input) trajectories before they are executed'),
        'type': 'boolean',
    }),
    ('saveRobotFeedbackLog', {
        'description': _('If True, will tell robotbridge to save trajectory files'),
        'type': 'boolean',
    }),
    ('loadRobotFeedbackLog', {
        'description': _('If True, will tell robotbridge to load the robot feedback log after trajectory ends'),
        'type': 'boolean',
    }),
    ('saveConcatenateTrajectoryLog', {
        'description': _('If True, will save trajectories used for inputs of concatenate trajectory functions'),
        'type': 'boolean',
    }),
    ('saveFilterTrajectoryLog', {
        'description': _('If True, will save trajectories used for filtering, such as SmartFilter'),
        'type': 'boolean',
    }),
    ('executionConnectingTrajReverseMult', {
        'description': _('Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally.'),
        'type': 'number',
    }),
    ('executionReverseRecoveryDistance', {
        'description': _('Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm).'),
        'type': 'number',
    }),
    ('webstackMountpoint', {
        'description': _('Location to load certain data from (e.g. template targets). Overrides localMediaDir.'),
        'type': 'string',
    }),
    ('localMediaDir', {
        'description': _('Directory to load local media from (e.g. for template targets). Overridden by webstackMountpoint. If preserveexternalrefs=True, this is used for some checks regardless.'),
        'type': 'string',
    }),
    ('viewer', {
        'description': _('If True, attempt to use viewer...?'),
        'type': 'boolean',
    }),
])

Internal_SetRobotClientParameters = OrderedDict([
    ('unit', components.unit),
    ('robotname', components.robotname),
    ('toolname', components.toolname),
    ('robotBridgeConnectionInfo', robotBridgeConnectionInfo),
    ('locationCollisionInfos', locationCollisionInfos),
])

Internal_TaskBaseInitParameters = MergeDicts(
    [
        pathPlannerParameters,
        smootherParameters,
        OrderedDict([
            ('slaverequestid', {
                'description': _('The requested planning slave'),
                'type': 'string',
            }),
            ('robot', {
                'description': _('Name of the robot'),
                'type': 'string',
            }),
            ('retimerplannername', {
                'description': _('Name of the retimer (planner). E.g.: LinearTrajectoryRetimer, ParabolicTrajectoryRetimer, CubicTrajectoryRetimer2, constraintparabolicsmoother. (Default: ParabolicTrajectoryRetimer2)'),
                'type': 'string',
            }),
            ('multidofinterp', {
                'description': _('Specifies how to interpolate multiple points. 0 is minimum acceleration, every point is independent. 2 is closer to robot makers with the switch times occurring simultaneously.'),
                'type': 'integer',
            }),
            ('vrcExtraClearance', {
                'description': _('The extra clearance that needs to be kept between VRC (Virtual Robot Controller) clearance and the simulated clearance. Depending on how well we can simulate robot maker controllers, this can be reduced. (Default: 0.02)'),
                'type': 'string',
            }),
        ])
    ],
    deepcopy=True,
)[0]
