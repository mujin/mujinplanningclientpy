# -*- coding: utf-8 -*-
# Copyright (C) 2013-2015 MUJIN Inc.
# Mujin controller client for bin picking task

# mujin imports
from .realtimerobotplanningclient import RealtimeRobotPlanningClient

# logging
import logging
log = logging.getLogger(__name__)


class HandEyeCalibrationPlanningClient(RealtimeRobotPlanningClient):
    """Mujin planning client for the HandEyeCalibration task"""

    tasktype = 'handeyecalibration'

    robot = None

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(self, robot, **kwargs):
        """Connects to the Mujin controller, initializes HandEyeCalibration task and sets up parameters

        Args:
            robot (str): Name of the robot, e.g. VP-5243I
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: handeyecalibration
            ctx (zmq.Context, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            slaverequestid:
            controllerip (str): IP or hostname of the mujin controller, e.g. 172.17.0.2 or controller123
            controllerurl (str, optional): (Deprecated. Use controllerip instead) URL of the mujin controller, e.g. http://controller14.
            controllerusername (str): Username for the Mujin controller, e.g. testuser
            controllerpassword (str): Password for the Mujin controller
            scenepk (str, optional): Primary key (pk) of the scene, e.g. irex_demo.mujin.dae
            callerid (str, optional): Caller identifier to send to server on every command
            ignoredArgs: Additional keyword args are not used, but allowed for easy initialization from a dictionary
        """
        super(HandEyeCalibrationPlanningClient, self).__init__(tasktype=self.tasktype, **kwargs)
        self.robot = robot


    #
    # Commands
    #

    def ComputeCalibrationPoses(self, primarySensorSelectionInfo, secondarySensorSelectionInfos, numsamples, calibboardvisibility, calibboardLinkName=None, calibboardGeomName=None, timeout=3000, gridindex=None, **kwargs):
        """Compute a set of calibration poses that satisfy the angle constraints using latin hypercube sampling (or stratified sampling upon failure)

        Args:
            primarySensorSelectionInfo (dict): Selects the primary camera that everything will be calibrated against.
            secondarySensorSelectionInfos (list[dict]): Selects the secondary camera(s) (assumed to be nearby the primary sensor).
            numsamples (int): Number of samples to take. A reasonable number is often between 5 and 25.
            calibboardvisibility (dict):
            calibboardLinkName (str, optional):
            calibboardGeomName (str, optional):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3000)
            gridindex (int, optional): The index of the voxel
            toolname (str, optional):
            calibboardObjectName (str, optional):
            minPatternTiltAngle (float, optional): The minimum tilt of the pattern in degrees. Default: 10 degrees
            maxPatternTiltAngle (float, optional): The maximum tilt of the pattern in degrees. Default: 30 degrees
            dynamicEnvironmentState (list[dict], optional): The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.
            robot (str, optional): The name of the robot (modelName). If not specified - the value used for client initialization will be applied.
        """
        taskparameters = {
            'command': 'ComputeCalibrationPoses',
            'primarySensorSelectionInfo': primarySensorSelectionInfo,
            'secondarySensorSelectionInfos': secondarySensorSelectionInfos,
            'numsamples': numsamples,
            'calibboardvisibility': calibboardvisibility,
        }
        if calibboardLinkName is not None:
            taskparameters['calibboardLinkName'] = calibboardLinkName
        if calibboardGeomName is not None:
            taskparameters['calibboardGeomName'] = calibboardGeomName
        if gridindex is not None:
            taskparameters['gridindex'] = gridindex
        if self.robot is not None:
            taskparameters['robot'] = self.robot
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SampleCalibrationConfiguration(self, primarySensorSelectionInfo, secondarySensorSelectionInfos, gridindex, calibboardvisibility, calibboardLinkName=None, calibboardGeomName=None, timeout=3000, minPatternTiltAngle=None, maxPatternTiltAngle=None, **kwargs):
        """Sample a valid calibration pose inside the given voxel and find a corresponding IK solution.

        Args:
            primarySensorSelectionInfo (dict): Selects the primary camera that everything will be calibrated against.
            secondarySensorSelectionInfos (list[dict]): Selects the secondary camera(s) (assumed to be nearby the primary sensor).
            gridindex (int): The index of the voxel
            calibboardvisibility (dict):
            calibboardLinkName (str, optional):
            calibboardGeomName (str, optional):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3000)
            minPatternTiltAngle (float, optional): The minimum tilt of the pattern in degrees. Default: 10 degrees
            maxPatternTiltAngle (float, optional): The maximum tilt of the pattern in degrees. Default: 30 degrees
            toolname (str, optional):
            calibboardObjectName (str, optional):
            dynamicEnvironmentState (list[dict], optional): The dynamic objects in the environment that is to be used for planning/executing the task. A list of bodies.
            robot (str, optional): The name of the robot (modelName). If not specified - the value used for client initialization will be applied.

        Returns:
            dict: A dictionary with the structure:

                - vConfig (list[float]): The IK solution (joint angles) for the sample.
        """
        taskparameters = {
            'command': 'SampleCalibrationConfiguration',
            'primarySensorSelectionInfo': primarySensorSelectionInfo,
            'secondarySensorSelectionInfos': secondarySensorSelectionInfos,
            'gridindex': gridindex,
            'calibboardvisibility': calibboardvisibility,
        }
        if calibboardLinkName is not None:
            taskparameters['calibboardLinkName'] = calibboardLinkName
        if calibboardGeomName is not None:
            taskparameters['calibboardGeomName'] = calibboardGeomName
        if minPatternTiltAngle is not None:
            taskparameters['minPatternTiltAngle'] = minPatternTiltAngle
        if maxPatternTiltAngle is not None:
            taskparameters['maxPatternTiltAngle'] = maxPatternTiltAngle
        if self.robot is not None:
            taskparameters['robot'] = self.robot
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ReloadModule(self, **kwargs):
        return self.ExecuteCommand({
            'command': 'ReloadModule',
        }, **kwargs)
