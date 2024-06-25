# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Optional, Union, Literal # noqa: F401 # used in type check
    import realtimeitl3planningclient_types as types

# mujin imports
from . import zmq
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class RealtimeITL3PlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the RealtimeITL3 task"""

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
        tasktype='realtimeitlplanning3',
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
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: realtimeitlplanning3
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
    # Commands (generated from the spec)
    #

    def SetJointValues(self, jointvalues, robotname=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (list[float], Optional[str], float, Optional[types.SetJointValuesParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            jointvalues:
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetJointValues',
            'jointvalues': jointvalues,
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetITLState(self, robotname=None, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, **kwargs):
        # type: (Optional[str], float, bool, Optional[types.GetITLStateParametersDynamicEnvironmentState], Optional[int], Optional[types.GetITLStateParametersRobotBridgeConnectionInfo], Optional[list[types.GetITLStateParametersLocationCollisionInfosArrayElement]], Optional[Any]) -> Optional[Any]
        """
        Args:
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
        """
        taskparameters = {
            'command': 'GetITLState',
        }  # type: dict[str, Any]
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ExecuteTrajectory(self, identifier, trajectories, statevalues=None, stepping=False, istep=None, cycles=1, restorevalues=None, envclearance=15, robotspeed=None, robotaccelmult=None, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (Any, Any, Optional[list[Any]], Any, Optional[bool], Any, Optional[list[Any]], float, Optional[float], Optional[float], float, bool, Optional[types.ExecuteTrajectoryParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            identifier:
            trajectories:
            statevalues: (Default: None)
            stepping: (Default: False)
            istep: (Default: None)
            cycles: (Default: 1)
            restorevalues: (Default: None)
            envclearance: Environment clearance in millimeters. (Default: 15)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ExecuteTrajectory',
            'identifier': identifier,
            'trajectories': trajectories,
            'stepping': stepping,
        }  # type: dict[str, Any]
        if statevalues is not None:
            taskparameters['statevalues'] = statevalues
        if istep is not None:
            taskparameters['istep'] = istep
        if cycles != 1:
            taskparameters['cycles'] = cycles
        if restorevalues is not None:
            taskparameters['restorevalues'] = restorevalues
        if envclearance != 15:
            taskparameters['envclearance'] = envclearance
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ExecuteTrajectoryStep(self, reverse=False, envclearance=15, robotspeed=None, robotaccelmult=None, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (bool, Optional[float], Optional[float], Optional[float], float, bool, Optional[types.ExecuteTrajectoryStepParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            reverse: (Default: False)
            envclearance: Environment clearance in millimeters. (Default: 15)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ExecuteTrajectoryStep',
            'reverse': reverse,
        }  # type: dict[str, Any]
        if envclearance != 15:
            taskparameters['envclearance'] = envclearance
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def PauseExecuteTrajectory(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (float, bool, Optional[types.PauseExecuteTrajectoryParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'PauseExecuteTrajectory',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ResumeExecuteTrajectory(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None):
        # type: (float, bool, Optional[types.ResumeExecuteTrajectoryParametersDynamicEnvironmentState], Optional[int]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ResumeExecuteTrajectory',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ComputeRobotConfigsForCommandVisualization(self, executiongraph, commandindex=0, timeout=2, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (types.ComputeRobotConfigsForCommandVisualizationParametersExecutiongraph, int, float, bool, Optional[types.ComputeRobotConfigsForCommandVisualizationParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            executiongraph:
            commandindex: (Default: 0)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 2)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ComputeRobotConfigsForCommandVisualization',
            'executiongraph': executiongraph,
        }  # type: dict[str, Any]
        if commandindex != 0:
            taskparameters['commandindex'] = commandindex
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def ComputeRobotJointValuesForCommandVisualization(self, program, commandindex=0, timeout=2, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, int, float, bool, Optional[types.ComputeRobotJointValuesForCommandVisualizationParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            program:
            commandindex: (Default: 0)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 2)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ComputeRobotJointValuesForCommandVisualization',
            'program': program,
        }  # type: dict[str, Any]
        if commandindex != 0:
            taskparameters['commandindex'] = commandindex
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def PlotProgramWaypoints(self, timeout=1, fireandforget=True, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.PlotProgramWaypointsParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 1)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: True)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'PlotProgramWaypoints',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StartITLProgram(
        self,
        programName,  # type: str
        robotspeed=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        timeout=10,  # type: float
        fireandforget=False,  # type: bool
        dynamicEnvironmentState=None,  # type: Optional[types.StartITLProgramParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        robotname=None,  # type: Optional[str]
        toolname=None,  # type: Optional[str]
        robotBridgeConnectionInfo=None,  # type: Optional[types.StartITLProgramParametersRobotBridgeConnectionInfo]
        locationCollisionInfos=None,  # type: Optional[list[types.StartITLProgramParametersLocationCollisionInfosArrayElement]]
        allowGrabWithoutTemplateTarget=False,  # type: bool
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        disallowSteppingBackwardAfterGrabRelease=True,  # type: bool
        encoderConvergenceSpeedThresh=None,  # type: Optional[float]
        envclearance=None,  # type: Optional[float]
        envClearanceMultiplierForJittering=None,  # type: Optional[float]
        goaljitter=None,  # type: Optional[float]
        initialjitter=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        nmaxiterations=None,  # type: Optional[int]
        numTrajectoryBuffer=None,  # type: Optional[int]
        maxNumPlanThreads=None,  # type: Optional[int]
        maxiter=None,  # type: Optional[int]
        postprocessingnmaxiterations=None,  # type: Optional[int]
        postprocessingplanner=None,  # type: Optional[str]
        planningSmallestObjectSizeForCollision=8.0,  # type: float
        saveRobotFeedbackLog=False,  # type: bool
        savetrajectorylog=False,  # type: bool
        separateplanningfallback=None,  # type: Optional[bool]
        smootherParameters=None,  # type: Optional[types.SmoothingParameters]
        saveExecutedITL=None,  # type: Optional[bool]
        startline=0,  # type: Optional[int]
        stepping=False,  # type: bool
        steplength=None,  # type: Optional[float]
        toolposes=None,  # type: Optional[types.StartITLProgramParametersToolposes]
        usedynamicsconstraints=None,  # type: Optional[bool]
        defaultItlProgramParams=None,  # type: Optional[types.StartITLProgramParametersDefaultItlProgramParams]
        executionid=None,  # type: Optional[str]
        itlCacheMode=None,  # type: Optional[str]
        parameters=None,  # type: Optional[types.StartITLProgramParametersParameters]
        programCommit=None,  # type: Optional[str]
        restorescene=None,  # type: Optional[bool]
        stamp=None,  # type: Optional[float]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Args:
            programName:
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            allowGrabWithoutTemplateTarget: Deprecated. Only for backwards compatibility. (Default: False)
            departOffsetDir: Departure offset direction. mm (x,y,z) (Default: None)
            disallowSteppingBackwardAfterGrabRelease: Deprecated. Only for backwards compatibility. (Default: True)
            encoderConvergenceSpeedThresh: (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            envClearanceMultiplierForJittering: (Default: None)
            goaljitter: (Default: None)
            initialjitter: (Default: None)
            jitter: (Default: None)
            nmaxiterations: (Default: None)
            numTrajectoryBuffer: (Default: None)
            maxNumPlanThreads: (Default: None)
            maxiter: (Default: None)
            postprocessingnmaxiterations: (Default: None)
            postprocessingplanner: (Default: None)
            planningSmallestObjectSizeForCollision: The smallest object size for collision detection while planning. (Default: 8.0)
            saveRobotFeedbackLog: Save logs from each trajectory the robot executes with data including the encoder values of the robot, the current/torque values, and specific IO signal values. When this feature is enabled, the system can slow down a little since it is storing data. Some UI functions that display data need this feature to be enabled so that the correct data can be displayed. (Default: False)
            savetrajectorylog: True of False (Default: False)
            separateplanningfallback: (Default: None)
            smootherParameters: Parameters dealing with getting smoother paths for the robot planning. (Default: None)
            saveExecutedITL: (Default: None)
            startline: Line of the ITL program to start at. This setting may be ignored by the server. (Default: 0)
            stepping: If queue mode is "Stepping" (line-by-line execution) or not. (Default: False)
            steplength: (Default: None)
            toolposes: Tool poses for the robot. (Default: None)
            usedynamicsconstraints: (Default: None)
            defaultItlProgramParams: (Default: None)
            executionid: (Default: None)
            itlCacheMode: (Default: None)
            parameters: (Default: None)
            programCommit: (Default: None)
            restorescene: (Default: None)
            stamp: (Default: None)
        """
        taskparameters = {
            'command': 'StartITLProgram',
            'programName': programName,
            'unit': unit,
        }  # type: dict[str, Any]
        if robotspeed is not None:
            taskparameters['robotspeed'] = robotspeed
        if robotaccelmult is not None:
            taskparameters['robotaccelmult'] = robotaccelmult
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if allowGrabWithoutTemplateTarget != False:
            taskparameters['allowGrabWithoutTemplateTarget'] = allowGrabWithoutTemplateTarget
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if disallowSteppingBackwardAfterGrabRelease != True:
            taskparameters['disallowSteppingBackwardAfterGrabRelease'] = disallowSteppingBackwardAfterGrabRelease
        if encoderConvergenceSpeedThresh is not None:
            taskparameters['encoderConvergenceSpeedThresh'] = encoderConvergenceSpeedThresh
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if envClearanceMultiplierForJittering is not None:
            taskparameters['envClearanceMultiplierForJittering'] = envClearanceMultiplierForJittering
        if goaljitter is not None:
            taskparameters['goaljitter'] = goaljitter
        if initialjitter is not None:
            taskparameters['initialjitter'] = initialjitter
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if nmaxiterations is not None:
            taskparameters['nmaxiterations'] = nmaxiterations
        if numTrajectoryBuffer is not None:
            taskparameters['numTrajectoryBuffer'] = numTrajectoryBuffer
        if maxNumPlanThreads is not None:
            taskparameters['maxNumPlanThreads'] = maxNumPlanThreads
        if maxiter is not None:
            taskparameters['maxiter'] = maxiter
        if postprocessingnmaxiterations is not None:
            taskparameters['postprocessingnmaxiterations'] = postprocessingnmaxiterations
        if postprocessingplanner is not None:
            taskparameters['postprocessingplanner'] = postprocessingplanner
        if planningSmallestObjectSizeForCollision != 8.0:
            taskparameters['planningSmallestObjectSizeForCollision'] = planningSmallestObjectSizeForCollision
        if saveRobotFeedbackLog != False:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if savetrajectorylog != False:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if separateplanningfallback is not None:
            taskparameters['separateplanningfallback'] = separateplanningfallback
        if smootherParameters is not None:
            taskparameters['smootherParameters'] = smootherParameters
        if saveExecutedITL is not None:
            taskparameters['saveExecutedITL'] = saveExecutedITL
        if startline != 0:
            taskparameters['startline'] = startline
        if stepping != False:
            taskparameters['stepping'] = stepping
        if steplength is not None:
            taskparameters['steplength'] = steplength
        if toolposes is not None:
            taskparameters['toolposes'] = toolposes
        if usedynamicsconstraints is not None:
            taskparameters['usedynamicsconstraints'] = usedynamicsconstraints
        if defaultItlProgramParams is not None:
            taskparameters['defaultItlProgramParams'] = defaultItlProgramParams
        if executionid is not None:
            taskparameters['executionid'] = executionid
        if itlCacheMode is not None:
            taskparameters['itlCacheMode'] = itlCacheMode
        if parameters is not None:
            taskparameters['parameters'] = parameters
        if programCommit is not None:
            taskparameters['programCommit'] = programCommit
        if restorescene is not None:
            taskparameters['restorescene'] = restorescene
        if stamp is not None:
            taskparameters['stamp'] = stamp
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def StopITLProgram(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, finishCode=None, finishMessage=None, **kwargs):
        # type: (float, bool, Optional[types.StopITLProgramParametersDynamicEnvironmentState], Optional[int], Optional[str], Optional[str], Optional[Any]) -> Optional[Any]
        """
        Stops the ITL program

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            finishCode: Optional finish code to end the cycle with (if it doesn't end with something else beforehand). (Default: None)
            finishMessage: (Default: None)
        """
        taskparameters = {
            'command': 'StopITLProgram',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if finishCode is not None:
            taskparameters['finishCode'] = finishCode
        if finishMessage is not None:
            taskparameters['finishMessage'] = finishMessage
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GenerateExecutionGraph(self, programName, commandTimeout=_deprecated, totalTimeout=_deprecated, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, unit='mm', robotname=None, toolname=None, robotBridgeConnectionInfo=None, locationCollisionInfos=None, executionid=None, parameters=None, programCommit=None, **kwargs):
        # type: (str, float, float, float, bool, Optional[types.GenerateExecutionGraphParametersDynamicEnvironmentState], Optional[int], str, Optional[str], Optional[str], Optional[types.GenerateExecutionGraphParametersRobotBridgeConnectionInfo], Optional[list[types.GenerateExecutionGraphParametersLocationCollisionInfosArrayElement]], Optional[str], Optional[types.GenerateExecutionGraphParametersParameters], Optional[str], Optional[Any]) -> Optional[types.GenerateExecutionGraphReturns]
        """
        Generates a list of commands for the ITL program.

        Args:
            programName:
            commandTimeout: **deprecated** Currently unused. (Default: 0.2)
            totalTimeout: **deprecated** Currently unused. (Default: 1.0)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            robotname: Name of the robot (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            executionid: (Default: None)
            parameters: (Default: None)
            programCommit: (Default: None)
        """
        taskparameters = {
            'command': 'GenerateExecutionGraph',
            'programName': programName,
            'unit': unit,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if robotname is not None:
            taskparameters['robotname'] = robotname
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if executionid is not None:
            taskparameters['executionid'] = executionid
        if parameters is not None:
            taskparameters['parameters'] = parameters
        if programCommit is not None:
            taskparameters['programCommit'] = programCommit
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def PopulateTargetInContainer(self, locationName, populateTargetUri, populateFnName, containerMetaData=None, timeout=20, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, Literal['mujinplanningcommon.planningutil.populateutil.container_filling_util.PopulatePensInBulkTray'], Optional[types.PopulateTargetInContainerParametersContainerMetaData], float, Optional[types.PopulateTargetInContainerParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Populates targets in the container using populateFn.

        Args:
            locationName:
            populateTargetUri:
            populateFnName:
            containerMetaData: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 20)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'PopulateTargetInContainer',
            'locationName': locationName,
            'populateTargetUri': populateTargetUri,
            'populateFnName': populateFnName,
        }  # type: dict[str, Any]
        if containerMetaData is not None:
            taskparameters['containerMetaData'] = containerMetaData
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

