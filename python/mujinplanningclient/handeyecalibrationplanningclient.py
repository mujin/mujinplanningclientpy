# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Dict, List, Optional, Tuple, Union # noqa: F401 # used in type check
    import handeyecalibrationplanningclient_types as types

# mujin imports
from . import zmq
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class HandEyeCalibrationPlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the HandEyeCalibration task"""

    robot = None  # type: str # type: ignore

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
        tasktype='handeyecalibration',
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
        # type: (str, int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
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
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: handeyecalibration
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
        self.robot = robotname
        super(HandEyeCalibrationPlanningClient, self).__init__(
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

    def ComputeCalibrationPoses(self, primarySensorSelectionInfo, secondarySensorSelectionInfos, numsamples, calibboardvisibility, calibboardLinkName=None, calibboardGeomName=None, timeout=3000, gridindex=None, toolname=None, calibboardObjectName=None, minPatternTiltAngle=None, maxPatternTiltAngle=None, dynamicEnvironmentState=None, robot=None, debuglevel=None, **kwargs):
        # type: (types.SensorSelectionInfo, list[types.SensorSelectionInfo], int, types.ComputeCalibrationPosesParametersCalibboardvisibility, Optional[str], Optional[str], float, Optional[int], Optional[str], Optional[str], Optional[float], Optional[float], Optional[types.ComputeCalibrationPosesParametersDynamicEnvironmentState], Optional[str], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Compute a set of calibration poses that satisfy the angle constraints using latin hypercube sampling (or stratified sampling upon failure)

        Args:
            primarySensorSelectionInfo: Selects the primary camera that everything will be calibrated against.
            secondarySensorSelectionInfos: Selects the secondary camera(s) (assumed to be near the primary sensor).
            numsamples: Number of samples to take. A reasonable number is often between 5 and 25.
            calibboardvisibility:
            calibboardLinkName: (Default: None)
            calibboardGeomName: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 3000)
            gridindex: The index of the voxel (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            calibboardObjectName: (Default: None)
            minPatternTiltAngle: The minimum tilt of the pattern in degrees. Default: 10 degrees (Default: None)
            maxPatternTiltAngle: The maximum tilt of the pattern in degrees. Default: 30 degrees (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            robot: The name of the robot (modelName). If not specified - the value used for client initialization will be applied. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ComputeCalibrationPoses',
            'primarySensorSelectionInfo': primarySensorSelectionInfo,
            'secondarySensorSelectionInfos': secondarySensorSelectionInfos,
            'numsamples': numsamples,
            'calibboardvisibility': calibboardvisibility,
        }  # type: dict[str, Any]
        if calibboardLinkName is not None:
            taskparameters['calibboardLinkName'] = calibboardLinkName
        if calibboardGeomName is not None:
            taskparameters['calibboardGeomName'] = calibboardGeomName
        if gridindex is not None:
            taskparameters['gridindex'] = gridindex
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if calibboardObjectName is not None:
            taskparameters['calibboardObjectName'] = calibboardObjectName
        if minPatternTiltAngle is not None:
            taskparameters['minPatternTiltAngle'] = minPatternTiltAngle
        if maxPatternTiltAngle is not None:
            taskparameters['maxPatternTiltAngle'] = maxPatternTiltAngle
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if robot is not None:
            taskparameters['robot'] = robot
        elif self.robot is not None:
            taskparameters['robot'] = self.robot
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)
    def SampleCalibrationConfiguration(self, primarySensorSelectionInfo, secondarySensorSelectionInfos, gridindex, calibboardvisibility, calibboardLinkName=None, calibboardGeomName=None, timeout=3000, minPatternTiltAngle=None, maxPatternTiltAngle=None, toolname=None, calibboardObjectName=None, dynamicEnvironmentState=None, robot=None, debuglevel=None, **kwargs):
        # type: (types.SensorSelectionInfo, list[types.SensorSelectionInfo], int, types.SampleCalibrationConfigurationParametersCalibboardvisibility, Optional[str], Optional[str], float, Optional[float], Optional[float], Optional[str], Optional[str], Optional[types.SampleCalibrationConfigurationParametersDynamicEnvironmentState], Optional[str], Optional[int], Optional[Any]) -> Optional[types.SampleCalibrationConfigurationReturns]
        """
        Sample a valid calibration pose inside the given voxel and find a corresponding IK solution.

        Args:
            primarySensorSelectionInfo: Selects the primary camera that everything will be calibrated against.
            secondarySensorSelectionInfos: Selects the secondary camera(s) (assumed to be near the primary sensor).
            gridindex: The index of the voxel
            calibboardvisibility:
            calibboardLinkName: (Default: None)
            calibboardGeomName: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 3000)
            minPatternTiltAngle: The minimum tilt of the pattern in degrees. Default: 10 degrees (Default: None)
            maxPatternTiltAngle: The maximum tilt of the pattern in degrees. Default: 30 degrees (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            calibboardObjectName: (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            robot: The name of the robot (modelName). If not specified - the value used for client initialization will be applied. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SampleCalibrationConfiguration',
            'primarySensorSelectionInfo': primarySensorSelectionInfo,
            'secondarySensorSelectionInfos': secondarySensorSelectionInfos,
            'gridindex': gridindex,
            'calibboardvisibility': calibboardvisibility,
        }  # type: dict[str, Any]
        if calibboardLinkName is not None:
            taskparameters['calibboardLinkName'] = calibboardLinkName
        if calibboardGeomName is not None:
            taskparameters['calibboardGeomName'] = calibboardGeomName
        if minPatternTiltAngle is not None:
            taskparameters['minPatternTiltAngle'] = minPatternTiltAngle
        if maxPatternTiltAngle is not None:
            taskparameters['maxPatternTiltAngle'] = maxPatternTiltAngle
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if calibboardObjectName is not None:
            taskparameters['calibboardObjectName'] = calibboardObjectName
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if robot is not None:
            taskparameters['robot'] = robot
        elif self.robot is not None:
            taskparameters['robot'] = self.robot
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ReloadModule(self, **kwargs):
        return self.ExecuteCommand({
            'command': 'ReloadModule',
        }, **kwargs)
