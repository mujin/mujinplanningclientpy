# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

from . import planningclient

# logging
import logging
log = logging.getLogger(__name__)

class MobileRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the multi-agent mobile robot path planner"""

    TASK_TYPE = 'mobilerobotplanning'

    def __init__(self, **kwargs):
        """Connects to the Mujin controller, initializes task"""
        super(MobileRobotPlanningClient, self).__init__(tasktype=self.TASK_TYPE, **kwargs)

    def GetTaskGraph(self):
        """Get the current state of the task graph"""
        taskparameters = {
            'command': 'GetTaskGraph',
        }
        return self.ExecuteCommand(taskparameters)

    def QueueTasks(self, newTasks):
        """Append new tasks to the task graph"""
        taskparameters = {
            'command': 'QueueTasks',
            'newTasks': newTasks,
        }
        return self.ExecuteCommand(taskparameters)

    def ClearTaskGraph(self):
        """Resets the current task graph. Robots will stop after finishing their current trajectories."""
        taskparameters = {
            'command': 'ClearTaskGraph',
        }
        return self.ExecuteCommand(taskparameters)

    def ForceReplan(self):
        """Triggers a forced replan of the mobile robot trajectories even if the task graph hasn't changed"""
        taskparameters = {
            'command': 'ForceReplan',
        }
        return self.ExecuteCommand(taskparameters)

    def ResumeCommandQueue(self):
        """Resume the bridge command queue for this planner"""
        taskparameters = {
            'command': 'ResumeCommandQueue',
        }
        return self.ExecuteCommand(taskparameters)
