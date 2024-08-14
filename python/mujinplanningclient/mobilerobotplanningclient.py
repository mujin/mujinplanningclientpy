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

    def ManageMobileRobotTasks(self, tasks, timeout=None, fireandforget=None, checkpreempt=True, blockwait=True, **kwargs):
        """
        This function will initialize the planning thread if it's not started.

        Arguments:
            tasks (list[dict]): list of MobileRobotTask changes.
        """
        taskparameters = {
            'command': 'ManageMobileRobotTasks',
            'tasks': tasks,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, checkpreempt=checkpreempt, blockwait=blockwait)
