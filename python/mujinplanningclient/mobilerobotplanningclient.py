# -*- coding: utf-8 -*-
# Copyright (C) 2023 MUJIN Inc.

import time
from mujincommon import MujinExceptionBase

from . import planningclient, TimeoutError

# logging
import logging
log = logging.getLogger(__name__)


class MobileRobotPlanningClientError(MujinExceptionBase):
    pass


class MobileRobotPlanningExecutionError(MujinExceptionBase):
    pass


class MobileRobotPlanningTimeoutError(MujinExceptionBase):
    pass


class MobileRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the multi-agent mobile robot path planner"""

    TASK_TYPE = 'mobilerobotplanning'
    _defaultTaskCommandTimeout = None # Default timeout(sec) for execute task command. (Default: 4.0)

    def __init__(self, **kwargs):
        """Connects to the Mujin controller, initializes task"""
        super(MobileRobotPlanningClient, self).__init__(tasktype=self.TASK_TYPE, **kwargs)
        self._defaultTaskCommandTimeout = 4.0

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

    # TODO: Adding optional parameters only for dev purpose, should be spreaded out to arg list.
    def ManageTaskGraph(self, tasks=None, optionalParameters={}, timeout=10.0, blockwait=True, fireandforget=False, checkPreempt=True, **kwargs):
        """
        """
        taskparameters = {
            'command': 'ManageTaskGraph',
            'tasks': tasks,
        }
        taskparameters.update(optionalParameters)
        return self.SendAndReceiveTaskCommand(self._PrepareTaskCommand(taskparameters, **kwargs), blockwait=blockwait, fireandforget=fireandforget, checkPreempt=checkPreempt)

    # TODO: Adding optional parameters only for dev purpose, should be spreaded out to arg list.
    def StartMobileRobotPlanningThread(self, robotBridgeConnectionInfo=None, chargingParameters=None, saveDebugResouresEveryPlanningLoop=False, optionalParameters={}, timeout=10.0, blockwait=True, fireandforget=False, checkPreempt=True, **kwargs):
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
            'saveDebugResouresEveryPlanningLoop': saveDebugResouresEveryPlanningLoop,
        }
        taskparameters.update(optionalParameters)
        return self.SendAndReceiveTaskCommand(self._PrepareTaskCommand(taskparameters, **kwargs), blockwait=blockwait, fireandforget=fireandforget, checkPreempt=checkPreempt)

    #
    # Async command utility
    # Enable the not blockWait option for zmq socket.
    #
    def SendAndReceiveTaskCommand(self, command, timeout=None, blockwait=True, checkPreempt=True, fireandforget=False):
        """
        Args:
            command (dict):
            timeout (float, optional): (Default: None)
            blockwait (bool, optional): If True, will block and wait until function is done. Otherwise user will have to call ProcessResponse on their own. (Default: True)
            checkPreempt (bool, optional): Check preempt function (Default: True)
            fireandforget (bool, optional): The opposite of blockwait. If True, will not wait. (Default: False)

        Returns:
            The result of _ProcessTaskCommandResponse (if blockWait and not fireandforget) or response of socket SendCommand.

        Raises:
            MobileRobotPlanningTimeoutError
        """
        assert self._commandsocket is not None
        try:
            if timeout is None:
                timeout = self._defaultTaskCommandTimeout
            response = self._commandsocket.SendCommand(command, blockwait=blockwait, timeout=timeout, sendjson=True, recvjson=True, checkpreempt=checkPreempt, fireandforget=fireandforget)
            if blockwait and not fireandforget:
                return self._ProcessTaskCommandResponse(response, command=command)
            else:
                return response

        except TimeoutError as e:
            raise MobileRobotPlanningTimeoutError('Timeout (%s) to get response for command=%s, message=%s' % (timeout, command, str(e)))

    def WaitForTaskCommandResponse(self, timeout=None, command=None):
        """Waits for a response for a command sent on the RPC socket.

        Args:
            timeout: (Default: None)
            command (dict, optional): Command sent to robotbridge (Default: None)

        Raises:
            MobileRobotPlanningClientError
        """
        assert self._commandsocket is not None
        if not self._commandsocket.IsWaitingReply():
            raise MobileRobotPlanningClientError('Waiting on command %s when wait signal is not on.' % command)
        return self._ProcessTaskCommandResponse(self._commandsocket.ReceiveCommand(timeout=timeout), command=command)

    def _PrepareTaskCommand(self, taskparameters, slaverequestid=None, respawnopts=None, forcereload=False):
        """Prepare the command to RPC to planning task.

        Args:
            taskparameters (dict): Task parameters in json format
            slaverequestid (str): TODO
            respawnopts (dict): TODO
            forcereload (bool): If True, then force re-load the scene before executing the task

        Returns:
            dict: returns the RPC command.
        """
        if 'stamp' not in taskparameters:
            taskparameters['stamp'] = time.time()
        if slaverequestid is None:
            slaverequestid = self._slaverequestid

        command = {
            'fnname': 'RunCommand',
            'taskparams': {
                'tasktype': self.tasktype, # TODO: as right now the mobilerobotplanning is the only task type.
                'sceneparams': self._sceneparams,
                'taskparameters': taskparameters,
                'forcereload': forcereload,
            },
            'userinfo': self._userinfo,
            'slaverequestid': slaverequestid,
            'stamp': time.time(),
            'respawnopts': respawnopts,
        }
        if self._callerid is not None:
            command['callerid'] = self._callerid
            if 'callerid' not in taskparameters:
                taskparameters['callerid'] = self._callerid

        return command

    def _ProcessTaskCommandResponse(self, response, command=None):
        """Checks response from planning server. Raises MobileRobotPlanningExecutionError if error is received.

        Args:
            response (dict): Response from planning server. Should be deserialized from json before calling this function.
            command (dict, optional): Command sent to robotbridge (Default: None)

        Returns:
            dict: The field 'output' of 'response'

        Raises:
            MobileRobotPlanningExecutionError
        """

        error = planningclient.GetAPIServerErrorFromZMQ(response)
        if error is not None:
            log.warn('GetAPIServerErrorFromZMQ returned error for %r', response)
            raise MobileRobotPlanningExecutionError(error)
        if response is None:
            if command is not None:
                log.warn(u'got no response from task command %r', command)
            return None
        return response['output']

    # def Clone(self):
    #     newclient = MobileRobotPlanningClient(self._serverip, self._serverport, self._checkpreemptfn, self._ctx if self._ctxown is None else None, self._robotname, defaultQueueMode=self._defaultQueueMode, callerid=self._callerid, queueid=self._queueid)
    #     return newclient
