# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

from . import planningclient

import logging
log = logging.getLogger(__name__)

class MobileRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the multi-agent mobile robot path planner"""

    tasktype = 'mobilerobotplanning'

    def __init__(self, **kwargs):
        kwargs['tasktype'] = self.tasktype # override task type
        super(MobileRobotPlanningClient, self).__init__(**kwargs)

    def GetState(self, timeout=10, fireandforget=False, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10.0)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            unit (str, optional): The unit of the given values. (Default: 'mm')
            robotname (str, optional): Name of the robot
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            robotBridgeConnectionInfo (dict, optional): Information to set up a client to the robot bridge.
            locationCollisionInfos (dict, optional): List of external collision IOs to be computed and sent in realtime.
        """
        taskparameters = {
            'command': 'GetState',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    # TODO: Adding optional parameters only for dev purpose, should be spreaded out to arg list.
    def ManageMobileRobotTasks(self, tasks, slaverequestid=None, timeout=None, fireandforget=None, respawnopts=None, checkpreempt=True, forcereload=False, blockwait=True, **kwargs):
        """
        This function will initialize the planning thread if it's not started.
        If the robotBridgeConnectionInfo is not set to use=True, no robot movement will occur as
        there will be no way to fetch updated information from the robot bridge server.
        Arguments:
            tasks (list[dict]): list of MobileRobotTask.
            robotBridgeConnectionInfo: specified as a dictionary with the following fields:
                use: bool: Whether the planning task should connect to robot bridges. Defaults to false.
                host: string: Hostname of the robot bridge server to communicate with
                port: string: Port of the robot bridge server to communicate with
                queueid: string: Name of the queue ID to use. Defaults to the slave request id.
            chargingParameters: specified as a dictionary with the following fields:
                minimumChargeTimeS: int: A robot that begins charging will always charge for at least this duration
                lowPowerThresholdPercent: Battery percentage [0, 100) below which a robot will be considered for charging
                sufficientPowerThresholdPercent: Battery percentage [0, 100) above which a robot will yield a charger to another robot
                highPowerThresholdPercent: Battery percentage [0, 100) above which a robot will disengage from the charger
            saveDebugResourcesEveryPlanningLoop: bool, true to save debug resources every re-planning loop.
        """
        taskparameters = {
            'command': 'ManageMobileRobotTasks',
            'tasks': tasks,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, slaverequestid=slaverequestid, timeout=timeout, fireandforget=fireandforget, respawnopts=respawnopts, checkpreempt=checkpreempt, forcereload=forcereload, blockwait=blockwait)
