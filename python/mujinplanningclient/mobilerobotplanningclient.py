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

    def StartMobileRobotPlanningThread(self, robotBridgeConnectionInfo=None, chargingParameters=None, **kwargs):
        """
        Initialize the planning thread.
        If the robotBridgeConnectionInfo is not set to use=True, no robot movement will occur as
        there will be no way to fetch updated information from the robot bridge server.
        Robot bridge connection info may be specified as a dictionary with the following fields:
            use: bool: Whether the planning task should connect to robot bridges. Defaults to false.
            host: string: Hostname of the robot bridge server to communicate with
            port: string: Port of the robot bridge server to communicate with
            queueid: string: Name of the queue ID to use. Defaults to the slave request id.
        Charging parameters may be specified as a dictionary with the following fields:
            minimumChargeTimeS: int: A robot that begins charging will always charge for at least this duration
            lowPowerThresholdPercent: Battery percentage [0, 100) below which a robot will be considered for charging
            sufficientPowerThresholdPercent: Battery percentage [0, 100) above which a robot will yield a charger to another robot
            highPowerThresholdPercent: Battery percentage [0, 100) above which a robot will disengage from the charger
        Example usage:
        >>>cc.StartMobileRobotPlanningThread(robotBridgeConnectionInfo={'use': True}) # to start planning thread
        """
        # Some of the expected parameters here do not match the standard naming convention.
        # Check that no accidentally misnamed parameters exist.
        if robotBridgeConnectionInfo:
            assert 'queueId' not in robotBridgeConnectionInfo

        taskparameters = {
            'command': 'StartMobileRobotPlanningThread',
            'robotBridgeConnectionInfo': robotBridgeConnectionInfo,
            'chargingParameters': chargingParameters,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters)

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

    def ClearCompletedTasks(self):
        """
        Removes all finished tasks from the task graph that are not dependencies of active tasks.
        Returns the set of tasks that were deleted.
        """
        taskparameters = {
            'command': 'ClearCompletedTasks',
        }
        return self.ExecuteCommand(taskparameters)

    def ClearTasksById(self, taskIdList):
        """
        Removes all tasks in the given list of task IDs from the task graph.
        Returns the set of tasks that were deleted.
        If an id is not present in the task graph, the call will throw and no changes will be made.
        """
        taskparameters = {
            'command': 'ClearTasksById',
            'taskIdList': taskIdList,
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
