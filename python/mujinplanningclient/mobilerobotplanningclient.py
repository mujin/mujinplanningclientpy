# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Optional, Union, Literal # noqa: F401 # used in type check
    import mobilerobotplanningclient_types as types

# mujin imports
from . import zmq
from . import planningclient

# logging
import logging
log = logging.getLogger(__name__)


class MobileRobotPlanningClient(planningclient.PlanningClient):
    """Mujin planning client for the MobileRobot task"""

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
        tasktype='mobilerobot',
        ctx=None,
        slaverequestid=None,
        controllerip='',
        controllerurl='',
        controllerusername='',
        controllerpassword='',
        scenepk='',
        callerid='',
        **ignoredArgs
    ):
        # type: (int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes MobileRobot task and sets up parameters

        Args:
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: mobilerobot
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
        # Override task type
        tasktype = 'mobilerobotplanning'
        super(MobileRobotPlanningClient, self).__init__(
            taskzmqport=taskzmqport,
            taskheartbeatport=taskheartbeatport,
            taskheartbeattimeout=taskheartbeattimeout,
            tasktype=tasktype,
            ctx=ctx,
            slaverequestid=slaverequestid,
            controllerip=controllerip,
            controllerurl=controllerurl,
            controllerusername=controllerusername,
            controllerpassword=controllerpassword,
            scenepk=scenepk,
            callerid=callerid
        )


    #
    # Commands (generated from the spec)
    #

    def ManageMobileRobotTasks(self, tasks, timeout=None, fireandforget=None, checkpreempt=True, blockwait=True, debuglevel=None, **kwargs):
        # type: (list[types.ManageMobileRobotTasksParametersTasksArrayElement], Optional[float], Optional[bool], bool, bool, Optional[int], Optional[Any]) -> Optional[Any]
        """
        This function will start the planning thread if it's not started.

        Args:
            tasks:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: None)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: None)
            checkpreempt: (Default: True)
            blockwait: Same as fireandforget, except will be able to receive return later with WaitForCommandResponse. (Default: True)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ManageMobileRobotTasks',
            'tasks': tasks,
        }  # type: dict[str, Any]
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget, checkpreempt=checkpreempt, blockwait=blockwait)

