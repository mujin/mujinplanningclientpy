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

    def TestStart(self):
        robotBridgeConnectionInfo=[
            {
                "address": "127.0.0.1",
                "remoteBodyNamePrefix": "local",
                "envTransform": [1, 0, 0, 0, 0, 0 ,0],
                "environmentIds": [
                    "frame",
                ],
            },
            {
                "address": "10.2.48.10",
                "remoteBodyNamePrefix": "c1007",
                "envTransform": [1, 0, 0, 0, 0, 20, 0],
                "environmentIds": [
                    "frame",
                    "T800",
                ],
            },
            {
                "address": "10.2.12.21",
                "remoteBodyNamePrefix": "c1004",
                "envTransform": [1, 0, 0, 0, 0, -20, 0],
                "environmentIds": [],
            },
        ]
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestHand(self):
        robotBridgeConnectionInfo=[
            {
                "address": "127.0.0.1",
                "remoteBodyNamePrefix": "local",
                "envTransform": [0, 0, 0, 1, 0, 10 ,0],
                "environmentIds": [
                    "frame",
                ],
            },
            {
                "address": "10.2.12.21",
                "remoteBodyNamePrefix": "c1004",
                "envTransform": [0.707, 0, 0, 0.707, 0, -10, 0],
                "environmentIds": [],
            },
        ]
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestCell(self):
        robotBridgeConnectionInfo=[
            {
                "address": "127.0.0.1",
                "remoteBodyNamePrefix": "local",
                "envTransform": [0, 0, 0, 1, 0, 10 ,0],
                "environmentIds": [
                    "frame",
                ],
            },
            {
                "address": "10.2.12.21",
                "remoteBodyNamePrefix": "c1004",
                "envTransform": [0.707, 0, 0, 0.707, 0, -10, 0],
                "environmentIds": [],
            },
            {
                "address": "10.2.12.59",
                "remoteBodyNamePrefix": "detect_cell",
                "envTransform": [0.707, 0, 0, 0.707, 10, 0, 0],
                "environmentIds": [],
            },
        ]
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestStartLocal(self):
        robotBridgeConnectionInfo=[
            {
                "address": "127.0.0.1",
                "envTransform": [1, 0, 0, 0, 50, 0 ,0],
                "environmentIds": [
                    "frame",
                ],
            },
        ]
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestStartRemote(self):
        robotBridgeConnectionInfo=[
            {
                "address": "10.2.48.10",
                "envTransform": [1, 0, 0, 0, 0, 10, 10],
                "environmentIds": [
                    "frame",
                    "T800",
                ],
            },
        ]
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestConfig(self):
        self.StartRemoteViewerPlanningThread()

    def TestStop(self):
        self.StartRemoteViewerPlanningThread(robotBridgeConnectionInfo=[])
