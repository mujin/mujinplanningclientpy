# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

from . import planningclient

import logging
log = logging.getLogger(__name__)

class MobileRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the multi-agent mobile robot path planner"""

    tasktype = 'mobilerobotplanning'

    # TODO: Adding optional parameters only for dev purpose, should be spreaded out to arg list.
    def ManageMobileRobotTasks(self, tasks=None, timeout=10.0, blockwait=True, fireandforget=False, checkPreempt=True, **kwargs):
        """
        This function will initialize the planning thread if it's not started.
        If the robotBridgeConnectionInfo is not set to use=True, no robot movement will occur as
        there will be no way to fetch updated information from the robot bridge server.
        Arguments:
            tasks: list of MobileRobotTask.
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
        return self.ExecuteCommand(taskparameters, timeout=timeout, blockwait=blockwait, fireandforget=fireandforget, checkPreempt=checkPreempt)
