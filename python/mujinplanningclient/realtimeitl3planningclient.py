# -*- coding: utf-8 -*-
# Copyright (C) 2017 MUJIN Inc.
# Mujin planning client for ITL task (v3)

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Dict, List, Optional, Tuple, Union # noqa: F401 # used in type check
    from . import zmq

# mujin imports
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class RealtimeITL3PlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the RealtimeITL3 task"""

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        robotname='',
        robotspeed=None,
        robotaccelmult=None,
        envclearance=10.0,
        robotBridgeConnectionInfo=None,
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
        tasktype='realtimeitl3',
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
        # type: (str, Optional[float], Optional[float], float, Optional[str], int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes RealtimeITL3 task and sets up parameters

        Args:
            robotname (str, optional): Name of the robot, e.g. VP-5243I
            robotspeed (float, optional): Speed of the robot, e.g. 0.4
            robotaccelmult (float, optional): Optional multiplier for the robot acceleration.
            envclearance (float, optional): Environment clearance in millimeter, e.g. 20
            robotBridgeConnectionInfo (str, optional): dict holding the connection info for the robot bridge.
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: realtimeitl3
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
        
        super(RealtimeITL3PlanningClient, self).__init__(
            robotname=robotname,
            robotspeed=robotspeed,
            robotaccelmult=robotaccelmult,
            envclearance=envclearance,
            robotBridgeConnectionInfo=robotBridgeConnectionInfo,
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
    # Commands
    #


    def SetJointValues(self, jointvalues, robotname=None, timeout=10, **kwargs):
        # type: (List[float], Optional[str], float, Any) -> Any
        """
        Args:
            jointvalues (list[float]):
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
        """
        taskparameters = {
            'command': 'SetJointValues',
            'jointvalues': jointvalues,
        }  # type: Dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetITLState(self, robotname=None, timeout=10, fireandforget=False, **kwargs):
        # type: (Optional[str], float, bool, Any) -> Any
        """
        Args:
            robotname (str, optional): Name of the robot
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'GetITLState',
        }  # type: Dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ExecuteTrajectory(self, identifier, trajectories, statevalues=None, stepping=False, istep=None, cycles=1, restorevalues=None, envclearance=15, robotspeed=None, robotaccelmult=None, timeout=10, fireandforget=False):
        # type: (Any, Any, Any, Any, Any, Any, Any, float, Optional[float], Optional[float], float, bool) -> Any
        """
        Args:
            identifier:
            trajectories:
            statevalues:
            stepping: (Default: False)
            istep:
            cycles: (Default: 1)
            restorevalues:
            envclearance (float, optional): Environment clearance in millimeters. (Default: 15)
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ExecuteTrajectory',
            'identifier': identifier,
            'trajectories': trajectories,
            'stepping': stepping,
            'cycles': cycles,
            'envclearance': envclearance,
        }  # type: Dict[str, Any]
        if statevalues is not None:
            taskparameters['statevalues'] = statevalues
        if istep is not None:
            taskparameters['istep'] = istep
        if restorevalues is not None:
            taskparameters['restorevalues'] = restorevalues
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ExecuteTrajectoryStep(self, reverse=False, envclearance=15, robotspeed=None, robotaccelmult=None, timeout=10, fireandforget=False):
        # type: (bool, float, Optional[float], Optional[float], float, bool) -> Any
        """
        Args:
            reverse (bool, optional): (Default: False)
            envclearance (float, optional): Environment clearance in millimeters. (Default: 15)
            robotspeed (float, optional): Value in (0,1] defining the percentage of speed the robot should move at.
            robotaccelmult (float, optional): Value in (0,1] defining the percentage of acceleration the robot should move at.
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ExecuteTrajectoryStep',
            'reverse': reverse,
            'envclearance': envclearance,
        }  # type: Dict[str, Any]
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def PauseExecuteTrajectory(self, timeout=10, fireandforget=False):
        # type: (float, bool) -> Any
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'PauseExecuteTrajectory',
        }  # type: Dict[str, Any]
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResumeExecuteTrajectory(self, timeout=10, fireandforget=False):
        # type: (float, bool) -> Any
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ResumeExecuteTrajectory',
        }  # type: Dict[str, Any]
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ComputeRobotConfigsForCommandVisualization(self, executiongraph, commandindex=0, timeout=2, fireandforget=False, **kwargs):
        # type: (Any, Any, float, bool, Any) -> Any
        """
        Args:
            executiongraph:
            commandindex: (Default: 0)
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 2)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ComputeRobotConfigsForCommandVisualization',
            'executiongraph': executiongraph,
            'commandindex': commandindex,
        }  # type: Dict[str, Any]
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ComputeRobotJointValuesForCommandVisualization(self, program, commandindex=0, timeout=2, fireandforget=False, **kwargs):
        # type: (Any, Any, float, bool, Any) -> Any
        """
        Args:
            program:
            commandindex: (Default: 0)
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 2)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'ComputeRobotJointValuesForCommandVisualization',
            'program': program,
            'commandindex': commandindex,
        }  # type: Dict[str, Any]
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def PlotProgramWaypoints(self, timeout=1, fireandforget=True, **kwargs):
        # type: (float, bool, Any) -> Any
        """
        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 1)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: True)
        """
        taskparameters = {
            'command': 'PlotProgramWaypoints',
        }  # type: Dict[str, Any]
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StartITLProgram(self, programName, robotspeed=None, robotaccelmult=None, timeout=10, fireandforget=False, **kwargs):
        # type: (Any, Any, Any, float, bool, Any) -> Any
        """
        Args:
            programName:
            robotspeed:
            robotaccelmult:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'StartITLProgram',
            'programName': programName,
        }  # type: Dict[str, Any]
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StopITLProgram(self, timeout=10, fireandforget=False, **kwargs):
        # type: (float, bool, Any) -> Any
        """Stops the ITL program

        Args:
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'StopITLProgram',
        }  # type: Dict[str, Any]
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GenerateExecutionGraph(self, programName, commandTimeout=0.2, totalTimeout=1.0, timeout=10, fireandforget=False, **kwargs):
        # type: (Any, Any, Any, float, bool, Any) -> Any
        """Generates a list of commands for the ITL program.

        Args:
            programName:
            commandTimeout: (Default: 0.2)
            totalTimeout: (Default: 1.0)
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget (bool, optional): If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
        """
        taskparameters = {
            'command': 'GenerateExecutionGraph',
            'programName': programName,
            'commandTimeout': commandTimeout,
            'totalTimeout': totalTimeout,
        }  # type: Dict[str, Any]
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def PopulateTargetInContainer(self, locationName, populateTargetUri, populateFnName, containerMetaData=None, timeout=20, **kwargs):
        # type: (Any, Any, Any, Optional[Dict], float, Any) -> Any
        """Populates targets in the container using populateFn.

        Args:
            locationName:
            populateTargetUri:
            populateFnName:
            containerMetaData (dict, optional):
            timeout (float, optional): Time in seconds after which the command is assumed to have failed. (Default: 20)
        """
        taskparameters = {
            'command': 'PopulateTargetInContainer',
            'locationName': locationName,
            'populateTargetUri': populateTargetUri,
            'populateFnName': populateFnName,
        }  # type: Dict[str, Any]
        if containerMetaData is not None:
            taskparameters['containerMetaData'] = containerMetaData
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)
