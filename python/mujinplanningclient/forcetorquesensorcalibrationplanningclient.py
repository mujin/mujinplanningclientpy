# -*- coding: utf-8 -*-
# Copyright (C) 2024 Mujin, Inc.

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Dict, Optional # noqa: F401 # used in type check
    from . import zmq

# mujin imports
from .realtimerobotplanningclient import RealtimeRobotPlanningClient

class ForceTorqueSensorCalibrationPlanningClient(RealtimeRobotPlanningClient):
    """Mujin planning client for the ForceTorqueSensorCalibration task"""

    tasktype = 'forcetorquesensorcalibration'

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        robotname=None,
        robotspeed=None,
        robotaccelmult=None,
        envclearance=10.0,
        robotBridgeConnectionInfo=None,
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
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
        # type: (Optional[str], Optional[float], Optional[float], float, Optional[str], int, int, float, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes HandEyeCalibration task and sets up parameters

        Args:
            robotname (str, optional): Name of the robot, e.g. VP-5243I
            robotspeed (float, optional): Speed of the robot, e.g. 0.4
            robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
            envclearance (float, optional): Environment clearance in millimeter, e.g. 20
            robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
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
        super(ForceTorqueSensorCalibrationPlanningClient, self).__init__(
            robotname=robotname,
            robotspeed=robotspeed,
            robotaccelmult=robotaccelmult,
            envclearance=envclearance,
            robotBridgeConnectionInfo=robotBridgeConnectionInfo,
            taskzmqport=taskzmqport,
            taskheartbeatport=taskheartbeatport,
            taskheartbeattimeout=taskheartbeattimeout,
            tasktype=self.tasktype,
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
    # Commands
    #


    def StartCalibrationTrajectoriesComputation(self, ftSensorLinkName, timeout=3000, **kwargs):
        # type: (Optional[float], Any) -> Any
        """Compute a set of calibration poses

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3000)
        """
        taskparameters = {
            'command': 'StartCalibrationTrajectoriesComputation',
            'ftSensorLinkName': ftSensorLinkName,
        }  # type: Dict[str, Any]
        return super(ForceTorqueSensorCalibrationPlanningClient, self).ExecuteCommand(taskparameters, timeout=timeout, **kwargs)

    def StartCalibrationTrajectoryExecution(self, ftSensorLinkName, timeout=3000, **kwargs):
        # type: (Optional[float], Any) -> Any
        """Compute a set of calibration poses

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 3000)
        """
        taskparameters = {
            'command': 'StartCalibrationTrajectoryExecution',
        }  # type: Dict[str, Any]
        return super(ForceTorqueSensorCalibrationPlanningClient, self).ExecuteCommand(taskparameters, timeout=timeout, **kwargs)

    def GetCalibrationTrajectoryInfos(self, **kwargs):
        taskparameters = {
            'command': 'GetCalibrationTrajectoryInfos',
        }
        return super(ForceTorqueSensorCalibrationPlanningClient, self).ExecuteCommand(taskparameters, **kwargs)

    def GetObservationInfos(self, **kwargs):
        taskparameters = {
            'command': 'GetObservationInfos',
        }
        return super(ForceTorqueSensorCalibrationPlanningClient, self).ExecuteCommand(taskparameters, **kwargs)

    def GetCurrentStageInfo(self, **kwargs):
        taskparameters = {
            'command': 'GetCurrentStageInfo',
        }
        return super(ForceTorqueSensorCalibrationPlanningClient, self).ExecuteCommand(taskparameters, **kwargs)
