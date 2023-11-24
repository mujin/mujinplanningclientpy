# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

from . import planningclient

# logging
import logging
log = logging.getLogger(__name__)

class RemoteViewerClient(planningclient.PlanningClient):
    """Mujin viewer client for the multiple environments synchronization"""

    TASK_TYPE = 'remoteviewertask'

    def __init__(self, **kwargs):
        """Connects to the Mujin controller, initializes task"""
        super(RemoteViewerClient, self).__init__(tasktype=self.TASK_TYPE, **kwargs)

    def StartRemoteViewerThread(self, robotBridgeConnectionInfo=None, chargingParameters=None):
        """
        Initialize the viewer thread.
        If the robotBridgeConnectionInfo is an empty array, all the previous connection will be destroyed.
        Robot bridge connection info may be specified as an array of objects with the following fields:
            host: string: Hostname of the robot bridge server to communicate with
            port: string: Port of the robot bridge server to communicate with
            queueid: string: Name of the queue ID to use. Defaults to the slave request id
            username: string: Username for WebStack connection
            password: string: Password for WebStack connection
            remoteBodyNamePrefix: string: Prefix which will be used for every body of the environment in main environment
            ignoreBodyPrefixes: array: List of prefixes to exclude from synchronization
            environmentIds: array: Environment IDs used for initial sync of the remote environment
            envTransform: array: Seven floating point numbers specifiying transformation to be applied to the remote environment when shown in the main one
        Example usage:
        >>>cc.StartRemoteViewerThread(robotBridgeConnectionInfo=[{}]) # to start viewer thread for local environment
        """
        # Some of the expected parameters here do not match the standard naming convention.
        # Check that no accidentally misnamed parameters exist.
        if robotBridgeConnectionInfo:
            assert 'queueId' not in robotBridgeConnectionInfo

        taskparameters = {
            'command': 'StartRemoteViewerThread',
            'robotBridgeConnectionInfo': robotBridgeConnectionInfo,
        }
        return self.ExecuteCommand(taskparameters)

    def Start(self, robotBridgeConnectionInfo):
        self.StartRemoteViewerThread(robotBridgeConnectionInfo)

    def StartWithConfig(self):
        self.StartRemoteViewerThread()

    def Stop(self):
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=[])

    def TestLocal(self):
        robotBridgeConnectionInfo=[
            {
                "address": "127.0.0.1",
                "remoteBodyNamePrefix": "local",
            },
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestRemote(self):
        robotBridgeConnectionInfo=[
            {
                "address": "10.2.48.10",
                "remoteBodyNamePrefix": "c1007",
                "environmentIds": [
                    "frame",
                    "T800",
                ],
            },
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestBasic(self):
        robotBridgeConnectionInfo=[
            {
                "address": "127.0.0.1",
                "remoteBodyNamePrefix": "local",
                "envTransform": [1, 0, 0, 0, 0, 0 ,0],
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
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestItodenki(self):
        robotBridgeConnectionInfo=[
            {
                "address": "10.2.12.21",
                "remoteBodyNamePrefix": "itodenki",
            },
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestDepalletizing(self):
        robotBridgeConnectionInfo=[
            {
                "address": "10.2.12.44",
                "remoteBodyNamePrefix": "depalletizing",
            },
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestDemo(self):
        robotBridgeConnectionInfo=[
            {
                "address": "10.2.12.44",
                "remoteBodyNamePrefix": "depalletizing",
                "envTransform": [1, 0, 0, 0, 0, 4 ,0],
            },
            {
                "address": "10.2.12.21",
                "remoteBodyNamePrefix": "itodenki",
                "envTransform": [1, 0, 0, 0, 0, 0, 0],
                "ignoreBodyPrefixes": [
                    "obs",
                    "wall"
                ]
            },
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)

    def TestDetection(self):
        robotBridgeConnectionInfo=[
            {
                "address": "10.2.12.59",
                "remoteBodyNamePrefix": "detection",
            },
        ]
        self.StartRemoteViewerThread(robotBridgeConnectionInfo=robotBridgeConnectionInfo)
