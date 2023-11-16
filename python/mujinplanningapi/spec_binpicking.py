# -*- coding: utf-8 -*-
# Copyright (C) 2023 Mujin, Inc.

from collections import OrderedDict

from . import _
from mujincommon.dictutil import MergeDicts

from . import components
from . import components_binpicking


services = [
    ('PickAndPlace', {
        'description': _('Picks up an object with the targetnamepattern and places it down at one of the goals. First computes the entire plan from robot moving to a grasp and then moving to its destination, then runs it on the real robot. Task finishes once the real robot is at the destination.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('goaltype', {
                                    'deprecated': True,
                                    'description': _('Type of the goal, e.g. translationdirection5d or transform6d'),
                                    'isRequired': False,
                                    'type': 'string',
                                }),
                                ('envclearance', components.envclearance),
                                ('movetodestination', {
                                    'description': _('planning parameter'),
                                    'type': 'integer',
                                }),
                                ('goals', {
                                    'deprecated': True,
                                    'description': _('Flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]'),
                                    'isRequired': False,
                                    'type': 'string',
                                }),
                                ('approachoffset', {
                                    'description': _('Distance in millimeters to move straight to the grasp point, e.g. 30 mm'),
                                    'type': 'number',
                                }),
                                ('departoffsetdir', {
                                    'description': _('The direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50]'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('destdepartoffsetdir', {
                                    'description': _('The direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system.'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('freeinc', {
                                    'description': _('planning parameter'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('worksteplength', {
                                    'description': _('planning parameter'),
                                    'type': 'number',
                                }),
                                ('targetnamepattern', {
                                    'description': _('regular expression describing the name of the object. No default will be provided, caller must set this. See https://docs.python.org/2/library/re.html'),
                                    'type': 'string',
                                }),
                                ('deletetarget', {
                                    'description': _('whether to delete target after pick and place is done'),
                                    'type': 'integer',
                                }),
                                ('debuglevel', components.debuglevel),
                                ('densowavearmgroup', {
                                    'description': _('planning parameter'),
                                    'type': 'integer',
                                }),
                                ('regionname', components_binpicking.regionname),
                                ('cameranames', {
                                    'description': _('The names of the cameras to avoid occlusions with the robot'),
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                                ('robotspeed', components.robotspeed),
                                ('toolname', components.toolname),
                                ('leaveoffsetintool', {
                                    'description': _('If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.'),
                                    'type': 'integer',
                                }),
                                ('desttargetname', {
                                    'description': _('The destination target name where the destination goal ikparams come from'),
                                    'type': 'string',
                                }),
                                ('destikparamnames', {
                                    'description': _('A list of lists of ikparam names for the destinations of the target. Only destikparamnames[0] is looked at and tells the system to place the part in any of the ikparams in destikparamnames[0]'),
                                    'items': {
                                        'items': {
                                            'type': 'string',
                                        },
                                        'type': 'array',
                                    },
                                    'type': 'array',
                                }),
                                ('graspsetname', {
                                    'description': _('the name of the grasp set belong to the target objects to use for the target. Grasp sets are a list of ikparams'),
                                    'type': 'string',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StartPickAndPlaceThread', {
        'description': _('Start a background loop to continuously pick up objects with the targetnamepattern and place them down at the goals. The loop will check new objects arriving in and move the robot as soon as it finds a feasible grasp. The thread can be quit with StopPickPlaceThread.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('goals', {
                                    'description': _('flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]'),
                                    'type': 'array',
                                }),
                                ('goaltype', {
                                    'description': _('type of the goal, e.g. translationdirection5d'),
                                    'type': 'string',
                                }),
                                ('cameranames', {
                                    'description': _('the names of the cameras to avoid occlusions with the robot'),
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                                ('envclearance', components.envclearance),
                                ('movetodestination', {
                                    'description': _('planning parameter'),
                                    'type': 'integer',
                                }),
                                ('approachoffset', {
                                    'description': _('distance in millimeters to move straight to the grasp point, e.g. 30 mm'),
                                    'type': 'number',
                                }),
                                ('departoffsetdir', {
                                    'description': _('the direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50]'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('destdepartoffsetdir', {
                                    'description': _('the direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system.'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('worksteplength', {
                                    'description': _('planning parameter'),
                                    'type': 'number',
                                }),
                                ('targetnamepattern', {
                                    'description': _('regular expression describing the name of the object, no default will be provided, caller must set this. See https://docs.python.org/2/library/re.html'),
                                    'type': 'string',
                                }),
                                ('regionname', components_binpicking.regionname),
                                ('deletetarget', {
                                    'description': _('whether to delete target after pick and place is done'),
                                    'type': 'integer',
                                }),
                                ('debuglevel', components.debuglevel),
                                ('cycledests', {
                                    'description': _('When finished cycling through all destikparamnames, will delete all the targets and start from the first index again doing this for cycledests times. By default it is 1.'),
                                    'type': 'integer',
                                }),
                                ('desttargetname', {
                                    'description': _('The destination target name where the destination goal ikparams come from'),
                                    'type': 'string',
                                }),
                                ('destikparamnames', {
                                    'description': _('A list of lists of ikparam names for the ordered destinations of the target. destikparamnames[0] is where the first picked up part goes, desttargetname[1] is where the second picked up target goes.'),
                                    'items': {
                                        'items': {
                                            'type': 'string',
                                        },
                                        'type': 'array',
                                    },
                                    'type': 'array',
                                }),
                                ('leaveoffsetintool', {
                                    'description': _('If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.'),
                                    'type': 'integer',
                                }),
                                ('robotspeed', components.robotspeed),
                                ('toolname', components.toolname),
                                ('densowavearmgroup', {
                                    'description': _('robot parameters'),
                                    'type': 'integer',
                                }),
                                ('graspsetname', {
                                    'description': _('the name of the grasp set belong to the target objects to use for the target. Grasp sets are a list of ikparams'),
                                }),
                                ('useworkspaceplanner', {
                                    'description': _('If 1 is set, will try the workspace planner for moving the hand straight. If 2 is set, will try the RRT for moving straight. Can set 3 for trying both.'),
                                    'type': 'integer',
                                }),
                                ('forceStartRobotValues', {
                                    'description': _('planning loop should always start from these values rather than reading from robot'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('initiallyDisableRobotBridge', {
                                    'default': False,
                                    'description': _('If True, stops any communication with the robotbridge until robot bridge is enabled.'),
                                    'type': 'boolean',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StopPickPlaceThread', {
        'description': _('stops the pick and place thread started with StartPickAndPlaceThread'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('resetExecutionState', {
                                    'description': _('if True, then reset the order state variables. By default True'),
                                    'type': 'boolean',
                                }),
                                ('resetStatusPickPlace', {
                                    'description': _('if True, then reset the statusPickPlace field of hte planning slave.'),
                                    'type': 'boolean',
                                }),
                                ('finishCode', {
                                    'description': _("optional finish code to end the cycle with (if it doesn't end with something else beforehand)"),
                                    'type': 'string',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetPickPlaceStatus', {
        'description': _('Gets the status of the pick and place thread'),
        'returns': {
            'description': _('Status of the pick and place thread.'),
            'properties': OrderedDict([
                ('status', {
                    'description': _('One for the following. 0: not running, 1: no error, 2: error'),
                    'type': 'integer',
                }),
                ('error', {
                    'type': 'string',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('ComputeIK', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('direction', {
                                    'description': _('grasp (but basically the just the ikparam) direction in world cooordinates'),
                                    'type': 'array',
                                }),
                                ('angle', {
                                    'description': _('grasp (but basically the just the ikparam) angle in world cooordinates'),
                                    'type': 'number',
                                }),
                                ('iktype', {
                                    'description': _('grasp (but basically the just the ikparam)'),
                                    'type': 'string',
                                }),
                                ('freeincvalue', {
                                    'description': _('The discretization of the free joints of the robot when computing ik.'),
                                    'type': 'number',
                                }),
                                ('quaternion', components.quaternion),
                                ('toolname', components.toolname),
                                ('limit', {
                                    'description': _('number of solutions to return'),
                                    'type': 'integer',
                                }),
                                ('preshape', {
                                    'description': _('If the tool has fingers after the end effector, specify their values. The gripper DOFs come from **gripper_dof_pks** field from the tool.'),
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('filteroptions', components.filteroptions),
                                ('translation', components.translation),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('solutions', {
                    'description': _('array of IK solutions (each of which is an array of DOF values), sorted by minimum travel distance and truncated to match the limit'),
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('InitializePartsWithPhysics', {
        'description': _('Start a physics simulation where the parts drop down into the bin. The method returns as soon as the physics is initialized, user has to wait for the "duration" or call StopPhysicsThread command.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('numtargets', {
                                    'description': _('the number of targets to create'),
                                }),
                                ('basename', {
                                    'description': _("The basename to give to all the new target names. Numbers are suffixed at the end, like basename+'0134'. If not specified, will use a basename derived from the targeturi."),
                                }),
                                ('targeturi', {
                                    'description': _('the target uri to initialize the scene with'),
                                }),
                                ('deleteprevious', {
                                    'description': _('if True, will delete all the previous targets in the scene. By default this is True.'),
                                }),
                                ('forcegravity', {
                                    'description': _("if not None, the gravity with which the objects should fall with. If None, then uses the scene's gravity"),
                                }),
                                ('duration', {
                                    'description': _('the duration in seconds to continue the physics until it is stopped.'),
                                }),
                                ('regionname', MergeDicts(
                                    [
                                        components_binpicking.regionname,
                                        {
                                            'description': _('The container name to drop the parts into.'),
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StopPhysicsThread', {
        'description': _('stops the physics simulation started with InitializePartsWithPhysics'),
        'returns': {},
    }),
    ('JitterPartUntilValidGrasp', {
        'description': _("Select a part that wasn't able to be grasped and jitter its location such that a grasp set is found for it that will take it to the destination."),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('jitterdist', {
                                    'description': _('Amount to jitter the target object translation by'),
                                }),
                                ('leaveoffsetintool', {
                                    'description': _('If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0.'),
                                }),
                                ('jitteriters', {
                                    'description': _('Number of times to try jittering before giving up.'),
                                }),
                                ('toolname', components.toolname),
                                ('desttargetname', {
                                    'description': _("The destination target name where the destination goal ikparams come from. If no name is specified, then robot won't consider putting the target into the destination when it searches for grasps."),
                                }),
                                ('destikparamnames', {
                                    'description': _('A list of lists of ikparam names for the ordered destinations of the target. destikparamnames[0] is where the first picked up part goes, desttargetname[1] is where the second picked up target goes.'),
                                    'items': {
                                        'items': {
                                            'type': 'string',
                                        },
                                        'type': 'array',
                                    },
                                    'type': 'array',
                                }),
                                ('graspsetname', components.graspsetname),
                                ('jitterangle', {
                                    'description': _("Amount to jitter the target object's orientation angle"),
                                }),
                                ('departoffsetdir', {
                                    'description': _('The depart distance for simulating full grasp.'),
                                }),
                                ('targetname', components.targetname),
                                ('approachoffset', {
                                    'description': _('The approach distance for simulating full grasp.'),
                                }),
                                ('destdepartoffsetdir', {
                                    'description': _('the direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system.'),
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('If failed, an empty dictionary. If succeeded, a dictionary with the following keys:\n- translation: the new translation of the target part\n- quaternion: the new quaternion of the target part\n- jointvalues: robot joint values that are grasping the part (fingers are at their preshape).\n- graspname: the grasp name used for jointvalues. If empty, then no grasp was found.\n- destikname: the name of the destination ikparam that was chosen with the grasp\n- destjointvalues: robot joint values at one of the specified destinations (fingers are at their final positions).\n- desttranslation: the new translation of the target part\n- destquaternion: the new quaternion of the target part\n'),
        },
    }),
    ('MoveToDropOff', {
        'description': _('Moves the robot to desired joint angles.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('dropOffInfo', {
                                    'isRequired': True,
                                }),
                                ('execute', MergeDicts(
                                    [
                                        components.execute,
                                        {
                                            'default': 1,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('envclearance', components.envclearance),
                                ('startvalues', {
                                    'items': {
                                        'type': 'number',
                                    },
                                    'type': 'array',
                                }),
                                ('robotname', components.robotname),
                                ('robotspeed', components.robotspeed),
                                ('robotaccelmult', components.robotaccelmult),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('IsRobotOccludingBody', {
        'description': _('returns if the robot is occluding body in the view of the specified camera'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('bodyname', {
                                    'description': _('Name of the object'),
                                    'isRequired': True,
                                }),
                                ('cameraname', {
                                    'description': _('Name of the camera'),
                                    'isRequired': True,
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _("The occlusion state in a json dictionary, e.g. {'occluded': 0}"),
            'type': 'object',
        },
    }),
    ('GetPickedPositions', {
        'description': _('returns the poses and the timestamps of the picked objects'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('unit', components.unit),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _("The positions and the timestamps of the picked objects in a json dictionary, info of each object has the format of quaternion (w,x,y,z) followed by x,y,z translation (in mm) followed by timestamp in milisecond e.g. {'positions': [[1,0,0,0,100,200,300,1389774818.8366449],[1,0,0,0,200,200,300,1389774828.8366449]]}\n"),
        },
    }),
    ('GetPickAndPlaceLog', {
        'description': _('Gets the recent pick-and-place log executed on the binpicking server. The internal server keeps the log around until the next Pick-and-place command is executed.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('startindex', {
                                    'description': _('Start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last element.'),
                                    'type': 'integer',
                                }),
                                ('num', {
                                    'description': _('Number of trajectories from startindex to return. If 0 will return all the trajectories starting from startindex'),
                                    'type': 'integer',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'description': _('A dictionary with keys, for example:\n{\n\n    total: 10\n    messages: [{\n\n            "message":"message1",\n            "type":"",\n            "level":0,\n            "data": {\n\n                "jointvalues":[0,0,0,0,0,0]\n            }\n        },\n        ...\n    ]\n}\n'),
        },
    }),
    ('MoveRobotOutOfCameraOcclusion', {
        'description': _('Moves the robot out of camera occlusion and deletes targets if it was in occlusion.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('regionname', components_binpicking.regionname),
                                ('robotspeed', components.robotspeed),
                                ('toolname', components.toolname),
                                ('cameranames', {
                                    'description': _('The names of the cameras to avoid occlusions with the robot'),
                                    'items': {
                                        'type': 'string',
                                    },
                                    'type': 'array',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('PausePickPlace', {
        'parameters': components.StandardPlanningServerRequestParameters + [
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
        ],
        'returns': {},
    }),
    ('ResumePickPlace', {
        'parameters': components.StandardPlanningServerRequestParameters + [
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
        ],
        'returns': {},
    }),
    ('SendStateTrigger', {
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('stateTrigger', {
                                    'description': _('a string that represents a unique trigger'),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetBinpickingState', {
        'parameters': components.StandardPlanningServerRequestParameters + [
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
        ],
        'returns': {},
    }),
    ('SetStopPickPlaceAfterExecutionCycle', {
        'description': _('Sets the cycle for stopping after the current pick cycle finishes.\n\nIf robot has not grabbed a part yet, then will stop the robot immediately.\nOn proper finish of the pick cycle, robot should go back to the finish position.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('finishCode', {
                                    'description': _("The finish code to end with. If not specified, will be 'FinishedCycleStopped'"),
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('PutPartsBack', {
        'description': _('Runs saved planningresult trajectories.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('trajectory', {
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('numparts', {
                                    'isRequired': True,
                                    'type': 'integer',
                                }),
                                ('toolname', MergeDicts(
                                    [
                                        components.toolname,
                                        {
                                            'type': 'string',
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('grippervalues', {
                                    'type': 'array',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GenerateGraspModelFromIkParams', {
        'description': _('Generates grasp model IK for given setup.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('graspsetname', MergeDicts(
                                    [
                                        components.graspsetname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('robotname', components.robotname),
                                ('targeturi', {
                                    'description': _("uri of target scene, e.g. '4902201402644.mujin.dae'"),
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('toolname', MergeDicts(
                                    [
                                        components.toolname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('CheckGraspModelIk', {
        'description': _('Checks if grasp model is generated for given setup.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('graspsetname', MergeDicts(
                                    [
                                        components.graspsetname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('targeturi', {
                                    'description': _("str. uri of target scene like 'mujin:4902201402644.mujin.dae'"),
                                    'isRequired': True,
                                }),
                                ('toolname', MergeDicts(
                                    [
                                        components.toolname,
                                        {
                                            'isRequired': True,
                                        }
                                    ],
                                    deepcopy=True,
                                )[0]),
                                ('ikparamnames', {
                                    'type': 'array',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SetCurrentLayoutDataFromPLC', {
        'description': _('Sets current layout from PLC.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('containername', {
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('containerLayoutSize', {
                                    'isRequired': True,
                                }),
                                ('ioVariableName', {
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                                ('destObstacleName', {
                                    'isRequired': True,
                                    'type': 'string',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('ClearVisualization', {
        'description': _('Clears visualization.'),
        'returns': {},
    }),
    ('GetPlanStatistics', {
        'description': _('Gets plan and execute statistics of the last pick and place'),
        'returns': {},
    }),
    ('SetCurrentLayoutDataSendOnObjectUpdateData', {
        'description': _('Sets currentLayoutDataSendOnObjectUpdateData structure'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('doUpdate', {
                                    'description': _('If True then currentLayoutData will be send on every ObjectUpdate, else currentLayoutDataSendOnObjectUpdate structure is reset'),
                                    'isRequired': True,
                                }),
                                ('containername', {
                                    'type': 'string',
                                }),
                                ('containerLayoutSize', {}),
                                ('ioVariableName', {}),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StartPackFormationComputationThread', {
        'description': _('Starts a background loop to copmute packing formation.'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('debuglevel', components.debuglevel),
                                ('toolname', components.toolname),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('StopPackFormationComputationThread', {
        'description': _('Stops the packing computation thread thread started with StartPackFormationComputationThread'),
        'returns': {},
    }),
    ('VisualizePackingState', {
        'description': _('Stops the packing computation thread thread started with StartPackFormationComputationThread'),
        'returns': {},
    }),
    ('VisualizePackFormationResult', {
        'description': _('Stops the packing computation thread thread started with StartPackFormationComputationThread'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('initializeCameraPosition', {
                                    'description': _('Reset camera position'),
                                    'type': 'boolean',
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('GetPackFormationSolution', {
        'description': _('Stops the packing computation thread thread started with StartPackFormationComputationThread'),
        'returns': {},
    }),
    ('GetPackItemPoseInWorld', {
        'parameters': components.StandardPlanningServerRequestParameters + [
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
        ],
        'returns': {},
    }),
    ('ManuallyPlacePackItem', {
        'description': _('Places an item according to the pack formation assuming the item is placed manually and updates robotbridge state'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('packFormationComputationResult', {}),
                                ('inputPartIndex', {}),
                                ('placeLocationNames', {}),
                                ('orderNumber', {}),
                                ('numLeftToPick', {}),
                                ('placedTargetPrefix', {}),
                                ('dynamicGoalsGeneratorParameters', {}),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {},
    }),
    ('SendPackFormationComputationResult', {
        'description': _('Stops the packing computation thread thread started with StartPackFormationComputationThread'),
        'returns': {},
    }),
    ('GetLatestPackFormationResultList', {
        'description': _('Gets latest pack formation computation result'),
        'returns': {},
    }),
    ('ClearPackingStateVisualization', {
        'description': _('Clears packing visualization'),
        'returns': {},
    }),
    ('ValidatePackFormationResultList', {
        'description': _('Validates pack formation result list and compute info (fillRatio, packageDimensions, packedItemsInfo, etc) about it.\n\nkwargs are expected to be packing parameters.\n'),
        'parameters': components.StandardPlanningServerRequestParameters + [
            {
                'name': 'taskparams',
                'schema': {
                    'type': 'object',
                    'properties': {
                        'taskparameters': {
                            'type': 'object',
                            'properties': OrderedDict([
                                ('packFormationResultList', {
                                    'isRequired': True,
                                }),
                            ]),
                        },
                    },
                },
            },
        ],
        'returns': {
            'properties': OrderedDict([
                ('validatedPackFormationResultList', {
                    'items': {
                        'properties': OrderedDict([
                            ('validationStatus', {}),
                            ('errorCode', {}),
                            ('errorDesc', {}),
                            ('packFormationResult', {
                                'description': _('Optional.'),
                            }),
                        ]),
                        'type': 'object',
                    },
                    'type': 'array',
                }),
            ]),
            'type': 'object',
        },
    }),
    ('ComputeSamePartPackResultBySimulation', {
        'description': _('Computes pack formation for single part type.'),
        'returns': {},
    }),
    ('HasDetectionObstacles', {
        'description': _('Checks to see if the detection obstacles have all arrived.'),
        'returns': {},
    }),
]

binpickingSpec = {
    'info': {
        'title': _('Binpicking'),
        'description': 'The Binpicking API of the Mujin Planning Server.',
        'mujinspecformatversion': '0.0.1',
    },
    'services': OrderedDict(services),
}