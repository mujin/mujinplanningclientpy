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

    def StartPlanningThread(self, robotBridgeConnectionInfo=None, chargingParameters=None):
        """
        Initialize the planning thread.
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
        """
        # Some of the expected parameters here do not match the standard naming convention.
        # Check that no accidentally misnamed parameters exist.
        if robotBridgeConnectionInfo:
            assert 'queueId' not in robotBridgeConnectionInfo

        taskparameters = {
            'command': 'StartPlanningThread',
            'robotBridgeConnectionInfo': robotBridgeConnectionInfo,
            'chargingParameters': chargingParameters,
        }
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
