# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

from . import planningclient

# logging
import logging
log = logging.getLogger(__name__)

class RemoteViewerPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the multi-agent mobile robot path planner"""

    TASK_TYPE = 'remoteviewerplanningtask'

    def __init__(self, **kwargs):
        """Connects to the Mujin controller, initializes task"""
        super(RemoteViewerPlanningClient, self).__init__(tasktype=self.TASK_TYPE, **kwargs)

    def StartRemoteViewerPlanningThread(self, robotBridgeConnectionInfo=None, chargingParameters=None):
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
        >>>cc.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo={'use': True}) # to start planning thread
        """
        # Some of the expected parameters here do not match the standard naming convention.
        # Check that no accidentally misnamed parameters exist.
        if robotBridgeConnectionInfo:
            assert 'queueId' not in robotBridgeConnectionInfo

        taskparameters = {
            'command': 'StartRemoteViewerPlanningThread',
            'robotBridgeConnectionInfo': robotBridgeConnectionInfo,
            # 'chargingParameters': chargingParameters,
        }
        return self.ExecuteCommand(taskparameters)

    def PrintHelloWorld(self):
        print("Hello World")
        taskparameters = {
            'command': 'PrintHelloWorld',
        }
        return self.ExecuteCommand(taskparameters)

    def TestStart(self):
        robotBridgeConnectionInfo=[{"use": True, "host": "127.0.0.1", "port": 7000}]
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestStop(self):
        robotBridgeConnectionInfo={"use": False}
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)
