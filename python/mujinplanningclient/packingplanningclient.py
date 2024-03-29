# -*- coding: utf-8 -*-
# Copyright (C) 2013-2015 MUJIN Inc.
# Mujin planning client for packing task

# mujin imports
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class PackingPlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the Packing task"""
    tasktype = 'packing'

    def __init__(self, **kwargs):
        """Connects to the Mujin controller, initializes Packing task and sets up parameters

        Args:
            regionname (str, optional): Name of the bin, e.g. container1
            robotname (str, optional): Name of the robot, e.g. VP-5243I
            robotspeed (float, optional): Speed of the robot, e.g. 0.4
            robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
            envclearance (str, optional): Environment clearance in millimeter, e.g. 20
            robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
            targetname (str, optional): Name of the target, e.g. plasticnut-center
            toolname (str, optional): Name of the manipulator, e.g. 2BaseZ
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'packing', 'handeyecalibration', 'itlrealtimeplanning3'. Default: packing
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
        super(PackingPlanningClient, self).__init__(tasktype=self.tasktype, **kwargs)

    def StartPackFormationComputationThread(self, timeout=10, debuglevel=4, toolname=None, fireandforget=False, **kwargs):
        """Starts a background loop to copmute packing formation.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            debuglevel (int, optional): Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: 4)
            toolname (str, optional): Name of the manipulator. Defaults to currently selected tool
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'StartPackFormationComputationThread',
            'debuglevel': debuglevel,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout, fireandforget=fireandforget)

    def StopPackFormationComputationThread(self, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)
        """
        taskparameters = {
            'command': 'StopPackFormationComputationThread',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)

    def VisualizePackingState(self, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)
        """
        taskparameters = {
            'command': 'VisualizePackingState',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)

    def GetPackFormationSolution(self, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)
        """
        taskparameters = {
            'command': 'GetPackFormationSolution',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)

    def SendPackFormationComputationResult(self, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """Stops the packing computation thread thread started with StartPackFormationComputationThread

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)
        """
        taskparameters = {
            'command': 'SendPackFormationComputationResult',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)

    def GetLatestPackFormationResultList(self, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """Gets latest pack formation computation result

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)
        """
        taskparameters = {
            'command': 'GetLatestPackFormationResultList',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)

    def ValidatePackFormationResultList(self, packFormationResultList, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """Validates pack formation result list and compute info (fillRatio, packageDimensions, packedItemsInfo, etc) about it.

        kwargs are expected to be packing parameters.

        Args:
            packFormationResultList:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)

        Returns:
            dict: A dictionary with the structure:

                - validatedPackFormationResultList (list[dict]): Contains a dictionary with the structure:

                    - validationStatus
                    - errorCode
                    - errorDesc
                    - packFormationResult: Optional.
        """
        taskparameters = {
            'command': 'ValidatePackFormationResultList',
            'packFormationResultList': packFormationResultList,
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)

    def ComputeSamePartPackResultBySimulation(self, timeout=100, **kwargs):
        """Computes pack formation for single part type.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 100)
        """
        taskparameters = {
            'command': 'ComputeSamePartPackResultBySimulation',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def HasDetectionObstacles(self, timeout=100, **kwargs):
        """Checks to see if the detection obstacles have all arrived.

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 100)
        """
        taskparameters = {
            'command': 'HasDetectionObstacles',
        }
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetPackingState(self, timeout=10, fireandforget=False, blockwait=True, **kwargs):
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            blockwait (bool, optional): Same as fireandforget, except will be able to receive return later with WaitForCommandResponse(Default: False)
        """
        taskparameters = {'command': 'GetPackingState'}
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, blockwait=blockwait)
