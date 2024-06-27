# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Optional, Union, Literal # noqa: F401 # used in type check
    import binpickingplanningclient_types as types

# mujin imports
from . import zmq
from . import realtimerobotplanningclient

# logging
import logging
log = logging.getLogger(__name__)


class BinpickingPlanningClient(realtimerobotplanningclient.RealtimeRobotPlanningClient):
    """Mujin planning client for the Binpicking task"""

    regionname = None  # type: Optional[str] # The default region for Pick/Place calls

    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        regionname=None,
        robotname=None,
        robotspeed=None,
        robotaccelmult=None,
        envclearance=10.0,
        robotBridgeConnectionInfo=None,
        targetname=None,
        toolname=None,
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
        tasktype='binpicking',
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
        # type: (Optional[str], str, Optional[float], Optional[float], float, Optional[str], Optional[str], Optional[str], int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes Binpicking task and sets up parameters

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
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: binpicking
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
        self.regionname = regionname
        super(BinpickingPlanningClient, self).__init__(
            robotname=robotname,
            robotspeed=robotspeed,
            robotaccelmult=robotaccelmult,
            envclearance=envclearance,
            robotBridgeConnectionInfo=robotBridgeConnectionInfo,
            targetname=targetname,
            toolname=toolname,
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

    def PickAndPlace(
        self,
        goaltype=_deprecated,  # type: str
        goals=_deprecated,  # type: str
        targetnamepattern=None,  # type: str
        approachoffset=30,  # type: float
        departoffsetdir=[0, 0, 50],  # type: tuple[float, float, float]
        destdepartoffsetdir=[0, 0, 30],  # type: tuple[float, float, float]
        deletetarget=0,  # type: int
        debuglevel=4,  # type: int
        movetodestination=1,  # type: int
        freeinc=[0.08],  # type: list[float]
        worksteplength=_deprecated,  # type: float
        densowavearmgroup=5,  # type: int
        regionname=None,  # type: Optional[str]
        cameranames=None,  # type: Optional[list[str]]
        envclearance=None,  # type: float
        toolname=None,  # type: Optional[str]
        robotspeed=None,  # type: Optional[float]
        timeout=1000,  # type: float
        leaveoffsetintool=None,  # type: Optional[int]
        desttargetname=None,  # type: Optional[str]
        destikparamnames=None,  # type: Optional[list[list[str]]]
        graspsetname=None,  # type: Optional[str]
        dynamicEnvironmentState=None,  # type: Optional[types.PickAndPlaceParametersDynamicEnvironmentState]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Picks up an object with the targetnamepattern and places it down at one of the goals. First computes the entire plan from robot moving to a grasp and then moving to its destination, then runs it on the real robot. Task finishes once the real robot is at the destination.

        Args:
            goaltype: **deprecated** Type of the goal, e.g. translationdirection5d or transform6d
            goals: **deprecated** Flat list of goals, e.g. two 5d ik goals: [380,450,50,0,0,1, 380,450,50,0,0,-1]
            targetnamepattern: regular expression describing the name of the object. No default will be provided, caller must set this. See https://docs.python.org/2/library/re.html (Default: None)
            approachoffset: Distance in millimeters to move straight to the grasp point, e.g. 30 mm (Default: 30)
            departoffsetdir: The direction and distance in mm to move the part in global frame (usually along negative gravity) after it is grasped, e.g. [0,0,50] (Default: [0, 0, 50])
            destdepartoffsetdir: The direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system. (Default: [0, 0, 30])
            deletetarget: whether to delete target after pick and place is done (Default: 0)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: 4)
            movetodestination: planning parameter (Default: 1)
            freeinc: planning parameter (Default: [0.08])
            worksteplength: **deprecated** planning parameter (Default: None)
            densowavearmgroup: planning parameter (Default: 5)
            regionname: Name of the region of the objects. (Default: None)
            cameranames: The names of the cameras to avoid occlusions with the robot (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 1000)
            leaveoffsetintool: If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0. (Default: None)
            desttargetname: The destination target name where the destination goal ikparams come from (Default: None)
            destikparamnames: A list of lists of ikparam names for the destinations of the target. Only destikparamnames[0] is looked at and tells the system to place the part in any of the ikparams in destikparamnames[0] (Default: None)
            graspsetname: The name of the grasp set that belongs that should be used to grasp the target. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
        """
        assert (targetnamepattern is not None)
        if regionname is None:
            regionname = self.regionname
        taskparameters = {
            'command': 'PickAndPlace',
            'targetnamepattern': targetnamepattern,
            'approachoffset': approachoffset,
            'departoffsetdir': departoffsetdir,
            'destdepartoffsetdir': destdepartoffsetdir,
            'deletetarget': deletetarget,
            'debuglevel': debuglevel,
            'movetodestination': movetodestination,
            'freeinc': freeinc,
            'envclearance': envclearance,
        }  # type: dict[str, Any]
        if densowavearmgroup != 5:
            taskparameters['densowavearmgroup'] = densowavearmgroup
        if regionname is not None:
            taskparameters['containername'] = regionname
        if cameranames is not None:
            taskparameters['cameranames'] = cameranames
        if leaveoffsetintool is not None:
            taskparameters['leaveoffsetintool'] = leaveoffsetintool
        if desttargetname is not None:
            taskparameters['desttargetname'] = desttargetname
        if destikparamnames is not None:
            taskparameters['destikparamnames'] = destikparamnames
        if graspsetname is not None:
            taskparameters['graspsetname'] = graspsetname
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, robotspeed=robotspeed, timeout=timeout)

    def StartPickAndPlaceThread(
        self,
        goaltype=None,  # type: Optional[Any]
        goals=None,  # type: Optional[Any]
        targetnamepattern=None,  # type: str
        approachoffset=30,  # type: types.ApproachGraspOffset
        departoffsetdir=[0, 0, 50],  # type: tuple[float, float, float]
        destdepartoffsetdir=[0, 0, 30],  # type: tuple[float, float, float]
        deletetarget=0,  # type: bool
        debuglevel=4,  # type: Literal[0, 1, 2, 3, 4, 5]
        movetodestination=1,  # type: Literal[0, 1]
        worksteplength=_deprecated,  # type: Optional[float]
        regionname=None,  # type: Any
        envclearance=None,  # type: Any
        toolname=None,  # type: Optional[str]
        robotspeed=None,  # type: Optional[Any]
        timeout=10,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDynamicEnvironmentState]
        allTargetsDifferentUri=None,  # type: Optional[bool]
        absMaxPlanningTimeToWait=None,  # type: Optional[float]
        addGraspGoalPairWorker=None,  # type: Optional[bool]
        alwaysPlanOutOfOcclusion=None,  # type: Optional[Literal[0, 1]]
        approachCurrentExceedThresholds=None,  # type: Optional[list[float]]
        approachCurrentExceedThresholdsDelta=None,  # type: Optional[list[float]]
        approachForceTorqueExceedThresholds=None,  # type: Optional[list[float]]
        approachForceTorqueExceedThresholdsDelta=None,  # type: Optional[list[float]]
        approachOffsetFromWalls=None,  # type: Optional[float]
        atStartPlanDynamicContentsNames=None,  # type: Optional[list[str]]
        automaticToolPosConstraintNotGrabbing=None,  # type: Optional[bool]
        automaticToolPosConstraintWhenGrabbing=None,  # type: Optional[bool]
        barcodeScanningInfo=None,  # type: Optional[types.BarcodeScanningInfo]
        binpickingDebugMode=None,  # type: Optional[Literal[0, 1, 2]]
        bodyNameToAvoidFinalCollision=None,  # type: Optional[str]
        bottomScanPlacementInfo=None,  # type: Optional[types.BottomScanPlacementInfo]
        cameraPlanningInfos=None,  # type: Optional[list[types.CameraPlanningInfo]]
        cameraOcclusionApplyGrabbedState=None,  # type: Optional[bool]
        cameraOcclusionOffset=None,  # type: Optional[float]
        cameraOcclusionPaddingTime=None,  # type: Optional[float]
        cameraOcclusionPaddingTimeStart=None,  # type: Optional[float]
        cameraOcclusionPaddingTimeEnd=None,  # type: Optional[float]
        cameraOcclusionUseLinkVisibility=None,  # type: Optional[bool]
        canPlaceInSourceOnRecover=None,  # type: Optional[bool]
        canPlaceInSourceOnBarcodeScanFail=None,  # type: Optional[bool]
        canSetIsPickable=None,  # type: Optional[bool]
        checkCameraOcclusionAtMidDest=None,  # type: Optional[bool]
        checkCollisionAtDestNames=None,  # type: Optional[list[str]]
        checkDestContainerEmptyOnArrivalNames=None,  # type: Optional[list[str]]
        checkExpectedDetectedHeightThreshold=None,  # type: Optional[float]
        checkForEndEffectorLowerThanGraspDist=None,  # type: Optional[float]
        checkObstacleNames=None,  # type: Optional[list[str]]
        checkPickContainerEmptyOnFinish=None,  # type: Optional[bool]
        checkPlaceContainerEmptyOnArrival=None,  # type: Optional[bool]
        clearanceTopDestContainer=None,  # type: Optional[float]
        clearanceTopSourceContainer=None,  # type: Optional[float]
        computePickContainerEmptyOnFinish=None,  # type: Optional[bool]
        computePickContainerDamagedOnFinish=None,  # type: Optional[bool]
        constraintToolInfo=None,  # type: Optional[types.ConstraintToolInfo]
        constraintToolPosToSource=None,  # type: Optional[bool]
        containerEmptyWaitTime=None,  # type: Optional[float]
        cycledests=None,  # type: Optional[int]
        deleteDynamicObjectsAfterFinish=None,  # type: Optional[bool]
        deleteTargetWhenPlacedInDest=None,  # type: Optional[Literal['DeleteInAll', 'KeepInAll']]
        deleteTargetFromSourceContainer=None,  # type: Optional[Literal[0, 1]]
        destikparamnames=None,  # type: Optional[list[str]]
        intermediateCycles=None,  # type: Optional[list[types.IntermediateCycleInfo]]
        deleteTargetsOnPieceLost=None,  # type: Optional[bool]
        deleteTargetsOnRecovery=None,  # type: Optional[bool]
        deleteTargetsEveryCycleSlowMode=None,  # type: Optional[bool]
        destBarcodeScanningInfo=None,  # type: Optional[types.DestBarcodeScanningInfo]
        destBarcodeScanningInfoPerContainer=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDestBarcodeScanningInfoPerContainer]
        destTargetAABBAlignIkParameters=None,  # type: Optional[types.DestTargetAABBAlignIkParameters]
        destTargetAnyBottomFaceRotationParameters=None,  # type: Optional[types.DestTargetAnyBottomFaceRotationParameters]
        destTargetCornerParameters=None,  # type: Optional[types.DestTargetCornerParameters]
        destTargetStackParameters=None,  # type: Optional[types.DestTargetStackParameters]
        dynamicGoalsParameters=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDynamicGoalsParameters]
        ensurePickVisibilityAtPlace=None,  # type: Optional[Literal['required', 'preferred', 'disabled']]
        graspApproachInfos=None,  # type: Optional[types.StartPickAndPlaceThreadParametersGraspApproachInfos]
        graspApproachInfosPerURI=None,  # type: Optional[types.StartPickAndPlaceThreadParametersGraspApproachInfosPerURI]
        graspDepartInfos=None,  # type: Optional[types.StartPickAndPlaceThreadParametersGraspDepartInfos]
        graspDepartAboveNearbyObstacles=None,  # type: Optional[bool]
        graspDepartAboveNearbyObstaclesMaxDist=None,  # type: Optional[float]
        departOffsetFromWalls=None,  # type: Optional[float]
        destApproachCurrentExceedThresholds=None,  # type: Optional[list[float]]
        destApproachCurrentExceedThresholdsDelta=None,  # type: Optional[list[float]]
        destApproachForceTorqueExceedThresholds=None,  # type: Optional[list[float]]
        destApproachForceTorqueExceedThresholdsDelta=None,  # type: Optional[list[float]]
        destApproachClearContainerTop=None,  # type: Optional[bool]
        destApproachInfos=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDestApproachInfos]
        destApproachAccelDecelScaleMultOnTargetMass=None,  # type: Optional[list[types.DestApproachAccelDecelScaleMultOnTargetMass]]
        destApproachAccelDecelMultOnTilt=None,  # type: Optional[float]
        destApproachSpeedMultOnTilt=None,  # type: Optional[float]
        destApproachSpeedMultOnUnknownSize=None,  # type: Optional[float]
        destcontainername=None,  # type: Optional[str]
        destcoordtype=None,  # type: Optional[types.AllDestCoordType]
        destDepartFutureCycleMerge=None,  # type: Optional[bool]
        destDepartInfos=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDestDepartInfos]
        destDepartSpeedMultOnUnknownSize=None,  # type: Optional[float]
        destDepartClearContainerTop=None,  # type: Optional[bool]
        destFilterByTargetOrientationThresh=None,  # type: Optional[float]
        destFilterByTargetOrientationThreshPerMass=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDestFilterByTargetOrientationThreshPerMass]
        destPriorityLabelDirectionValue=None,  # type: Optional[float]
        destTargetValidationJitterDist=None,  # type: Optional[float]
        detectionResultsMaxCacheTime=None,  # type: Optional[float]
        discardTargetIfJitterFails=None,  # type: Optional[int]
        discardTargetIfMergeAfterJitterFails=None,  # type: Optional[int]
        disablePlacedTargetsInPickLocationWhenPlanning=None,  # type: Optional[bool]
        disableTargetCheckOnGraspApproach=None,  # type: Optional[bool]
        disableUnchuck=None,  # type: Optional[bool]
        detectionTriggerMode=None,  # type: Optional[Literal['AutoOnChange', 'WaitTrigger', 'Continuous']]
        distanceMeasurementInfo=None,  # type: Optional[types.DistanceMeasurementInfo]
        doAccurateGraspDepart=None,  # type: Optional[bool]
        doAutoRecoveryOnPieceLost=None,  # type: Optional[bool]
        doAutoRecoveryOnRobotExecutionError=None,  # type: Optional[bool]
        doSimultaneousGripperPreshapingDuringApproach=None,  # type: Optional[bool]
        dropInDestInfo=None,  # type: Optional[types.DropInDestInfo]
        dropInDestInfoPerContainer=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDropInDestInfoPerContainer]
        dropOffParameters=None,  # type: Optional[types.StartPickAndPlaceThreadParametersDropOffParameters]
        dropOffsetFromCollision=None,  # type: Optional[float]
        dropTargetInSourceContainerBoxMult=None,  # type: Optional[float]
        dropTargetInDestContainerBoxMult=None,  # type: Optional[float]
        dropTargetInDestContainerZSafetyMult=None,  # type: Optional[float]
        dropTargetMaxDistanceThresold=None,  # type: Optional[float]
        dropTargetMaxDistanceXYThreshold=None,  # type: Optional[float]
        enableBodyNamesOnCameraPlan=None,  # type: Optional[list[str]]
        enableBodyNamesOnDestPlan=None,  # type: Optional[list[str]]
        enableBodyNamesOnStartPlan=None,  # type: Optional[list[str]]
        encoderConvergenceSpeedThresh=None,  # type: Optional[float]
        executionConnectingTrajDecelMult=None,  # type: Optional[float]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        executionEnvClearanceApproachConcatenate=None,  # type: Optional[float]
        executionMaxConcatenateSearchTime=None,  # type: Optional[float]
        executionConcatenateSearchDeltaTime=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        executionFilterFactorWhenGrabbing=None,  # type: Optional[float]
        executethread=None,  # type: Optional[bool]
        executeITLOnCompleteLayerInfo=None,  # type: Optional[types.StartPickAndPlaceThreadParametersExecuteITLOnCompleteLayerInfo]
        executionVerificationInfo=None,  # type: Optional[types.ExecutionVerificationInfo]
        needContainerResetMode=None,  # type: Optional[Literal['ResetOnLastTargetInOrder', 'ResetOnNoMoreTargets', 'WaitForPickCompletion']]
        feedbackDefaultJointThreshold=None,  # type: Optional[float]
        feedbackDefaultWorkspaceThreshold=None,  # type: Optional[float]
        feedbackDestJointThreshold=None,  # type: Optional[float]
        feedbackDestWorkspaceThreshold=None,  # type: Optional[float]
        feedbackGraspJointThreshold=None,  # type: Optional[float]
        feedbackGraspWorkspaceThreshold=None,  # type: Optional[float]
        finalPlanMode=None,  # type: Optional[Literal['none', '', 'cameraocclusion', 'config', 'configIgnoreOcclusion']]
        finalPlanRobotConfiguration=None,  # type: Optional[str]
        filterGraspByUpPlaceOrientation=None,  # type: Optional[bool]
        finalPlanWaitTime=None,  # type: Optional[float]
        forceGraspModel=None,  # type: Optional[bool]
        forcetargetname=None,  # type: Optional[str]
        forceTargetNamePattern=None,  # type: Optional[str]
        forceTargetUriCheck=None,  # type: Optional[bool]
        forceSecondPreshapeAfterFirstApproach=None,  # type: Optional[bool]
        forceSecondReleaseAfterFirstDestDepart=None,  # type: Optional[bool]
        forceWaitDestContainer=None,  # type: Optional[bool]
        getCorrectPlanningReport=None,  # type: Optional[bool]
        doSecondPreshapeAfterFirstApproach=None,  # type: Optional[bool]
        doSecondReleaseAfterFirstDestDepart=None,  # type: Optional[bool]
        releaseFingerOffsetAfterFirstDestDepart=None,  # type: Optional[list[float]]
        grabbedTargetValidationSignalsInfo=None,  # type: Optional[types.GrabbedTargetValidationSignalsInfo]
        graspApproachClearContainerTop=None,  # type: Optional[bool]
        graspDepartClearContainerTop=None,  # type: Optional[bool]
        graspDepartCurrentExceedThresholds=None,  # type: Optional[list[float]]
        graspDepartCurrentExceedThresholdsDelta=None,  # type: Optional[list[float]]
        graspApproachCollisionWallOffsetParameters=None,  # type: Optional[types.CollisionWallOffsetParameters]
        graspDepartAccelDecelScaleMultOnTargetMass=None,  # type: Optional[types.GraspDepartAccelDecelScaleMultOnTargetMass]
        graspDepartReverseRecoveryDistance=None,  # type: Optional[float]
        graspDepartForceTorqueExceedThresholds=None,  # type: Optional[list[float]]
        graspDepartForceTorqueExceedThresholdsDelta=None,  # type: Optional[list[float]]
        graspDepartCollisionWallOffsetParameters=None,  # type: Optional[types.CollisionWallOffsetParameters]
        graspPenetrationOnTiltDist=None,  # type: Optional[float]
        graspsetname=None,  # type: Optional[str]
        graspFilterByApproachOrientationThresh=None,  # type: Optional[float]
        graspTimeLimit=None,  # type: Optional[float]
        graspGoalPairCostMultipliers=None,  # type: Optional[types.GraspGoalPairCostMultipliers]
        graspPriorityMultipliers=None,  # type: Optional[types.GraspPriorityMultipliers]
        heightDeltaUpdateOnNoMeasurement=None,  # type: Optional[float]
        maxAcceptedDestPlanTrajTime=None,  # type: Optional[float]
        maxAcceptedFinishTrajTime=None,  # type: Optional[float]
        heightIsAlwaysUncertain=None,  # type: Optional[bool]
        maxCandidateMass=None,  # type: Optional[float]
        maxNumPackFormationSolutions=None,  # type: Optional[float]
        moveToMidDestSkipWhenAllRegistered=None,  # type: Optional[bool]
        ignoreDynamicObstaclesInGraspDepart=None,  # type: Optional[bool]
        ignoreFinishPosition=None,  # type: Optional[bool]
        ignoreFinishPositionUnlessPackFormationComplete=None,  # type: Optional[bool]
        ignoreIsPickable=None,  # type: Optional[bool]
        ignoreMovableRobotNames=None,  # type: Optional[list[str]]
        iksolvername=None,  # type: Optional[Literal['ikfast', 'ikfastcpp']]
        ikSolverParameters=None,  # type: Optional[types.IkSolverParameters]
        iktimelimit=None,  # type: Optional[float]
        ikTimeLimitForHighestPriority=None,  # type: Optional[float]
        inspectionFailDropOffInfo=None,  # type: Optional[types.InspectionFailDropOffInfo]
        initialDetectionValidationInfo=None,  # type: Optional[types.InitialDetectionValidationInfo]
        initialMoveRobotOufOfCameraOcclusion=None,  # type: Optional[bool]
        inputPlacedPartInfoOnArrivals=None,  # type: Optional[types.StartPickAndPlaceThreadParametersInputPlacedPartInfoOnArrivals]
        ioSignalsInfo=None,  # type: Optional[types.IoSignalsInfo]
        isGripperSyncExecPossible=None,  # type: Optional[bool]
        isTestMode=None,  # type: Optional[bool]
        isStopOnGripperPositionNotReached=None,  # type: Optional[bool]
        isStopOnObjectMassPropertiesMismatch=None,  # type: Optional[bool]
        isStopOnPieceLost=None,  # type: Optional[bool]
        isStopOnRobotExecutionError=None,  # type: Optional[bool]
        isStopOnTorqueLimitsError=None,  # type: Optional[bool]
        isStopOnTorqueLimitsErrorOutsidePickLocation=None,  # type: Optional[bool]
        isStopOnControllerError=None,  # type: Optional[bool]
        itlProgramNamesOnEvent=None,  # type: Optional[list[types.ITLExecutionInfo]]
        cyclePreconditionIOInfo=None,  # type: Optional[types.CyclePreconditionIOInfo]
        logmessagesmask=None,  # type: Optional[int]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        justInTimeToolChangePlanning=None,  # type: Optional[types.JustInTimeToolChangePlanningParameters]
        labelPlacingInfo=None,  # type: Optional[types.LabelPlacingInfo]
        labelerDirection=None,  # type: Optional[tuple[float, float, float]]
        localTargetDir=None,  # type: Optional[tuple[float, float, float]]
        logFailedTargetTimeout=None,  # type: Optional[float]
        logFailedTargetPriorityTimeout=None,  # type: Optional[float]
        maxAllowedTargetSize=None,  # type: Optional[tuple[float, float, float]]
        maxAllowedTargetSizeObjectName=None,  # type: Optional[str]
        maxConsideredCameraIkSolutions=None,  # type: Optional[int]
        maxDestIkSolutions=None,  # type: Optional[int]
        maxGraspIkSolutions=None,  # type: Optional[int]
        maxGraspsToConsider=None,  # type: Optional[int]
        maxFinalPlanIgnoreCount=None,  # type: Optional[int]
        maxFinalPlanIgnoreMinTargets=None,  # type: Optional[int]
        maxGraspIkSolutionsPerGrasp=None,  # type: Optional[int]
        maxIncidenceAngleOfIgnoreDestLinksForDestApproach=None,  # type: Optional[float]
        maxIncidenceAngleOfIgnoreDestLinksForDestApproachIgnoreDistance=None,  # type: Optional[float]
        maxStartFailuresForTargetGrasp=None,  # type: Optional[int]
        maxStartFailuresForTarget=None,  # type: Optional[int]
        maxLinearFailuresForTarget=None,  # type: Optional[int]
        maxLinearFailuresForTargetGrasp=None,  # type: Optional[int]
        maxDestFailuresForTargetGrasp=None,  # type: Optional[int]
        maxTransferFailuresForTarget=None,  # type: Optional[int]
        maxNumConsecutivePieceLost=None,  # type: Optional[int]
        maxNumConsecutiveCycleFailures=None,  # type: Optional[int]
        maxNumConsecutiveDistanceSensorFailures=None,  # type: Optional[int]
        maxNumConsecutiveVerificationFailures=None,  # type: Optional[int]
        maxNumConsecutiveRegistrationFailures=None,  # type: Optional[int]
        maxNumConsecutiveSplitterOverweightFailures=None,  # type: Optional[int]
        maxNumPlanningFailedIterations=None,  # type: Optional[int]
        maxPlanningCyclesToQueue=None,  # type: Optional[int]
        maxTimeForDecrease=None,  # type: Optional[float]
        maxTorqueMultForApproach=None,  # type: Optional[float]
        maxTorqueMultForDestApproach=None,  # type: Optional[float]
        mergeTrajectoryParametersForRecovery=None,  # type: Optional[types.MergeTrajectoryParameters]
        midDestCoordType=None,  # type: Optional[Literal['tool', 'target']]
        midDestIkparamNames=None,  # type: Optional[list[str]]
        midDestPassthroughVelocity=None,  # type: Optional[tuple[float, float, float]]
        midDestWaitTime=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxManipSpeed=None,  # type: Optional[float]
        maxnotifygrasps=None,  # type: Optional[int]
        maxnotifydests=None,  # type: Optional[int]
        maxTargetNeighborsOnFace=None,  # type: Optional[int]
        minGraspDepartCompleteRatio=None,  # type: Optional[float]
        minNumSameConsecutiveDetections=None,  # type: Optional[int]
        minViableRegionPlanParameters=None,  # type: Optional[types.MinViableRegionPlanParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        moveStraightParamsOnReplan=None,  # type: Optional[types.MoveStraightParameters]
        moveToMidDest=None,  # type: Optional[bool]
        multiPickInfo=None,  # type: Optional[types.MultiPickInfo]
        neighTargetThresh=None,  # type: Optional[float]
        neighTargetThreshForTorqueError=None,  # type: Optional[float]
        neighTargetThreshY=None,  # type: Optional[float]
        notifyStateSlaveZMQUri=None,  # type: Optional[str]
        numInitialDestinationResultsWorkers=None,  # type: Optional[int]
        numInitialGraspWorkers=None,  # type: Optional[int]
        numInputPlacedPartInfoOnArrival=None,  # type: Optional[int]
        numLogGraspPriorities=None,  # type: Optional[int]
        numTargetsToPlace=None,  # type: Optional[int]
        numThreads=None,  # type: Optional[int]
        numValidBodiesToTriggerWaitForCapturingBeforeGraspApproach=None,  # type: Optional[int]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        objectMassPropertiesCheckingInfo=None,  # type: Optional[types.ObjectMassPropertiesCheckingInfo]
        objectTypeDynamicParametersMap=None,  # type: Optional[types.ObjectTypeDynamicParameters]
        targetOverlapConstraintInfo=None,  # type: Optional[types.TargetOverlapConstraintInfo]
        packFormationComputationResult=None,  # type: Optional[types.PackFormation]
        packFormationParameters=None,  # type: Optional[types.PackFormationParameters]
        orderPackFormationName=None,  # type: Optional[str]
        packLocationName=None,  # type: Optional[str]
        passOnDropAtDestinationNames=None,  # type: Optional[list[str]]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        pickFailureDepartRetryNum=None,  # type: Optional[int]
        pickFailureDepartRetryWidth=None,  # type: Optional[float]
        pickLocationInfo=None,  # type: Optional[types.PickLocationInfo]
        pieceInspectionInfo=None,  # type: Optional[types.PieceInspectionInfo]
        placedTargetPrefix=None,  # type: Optional[str]
        placedTargetExtraPadding=None,  # type: Optional[tuple[float, float, float]]
        placedTargetRemainingTranslationRange=None,  # type: Optional[types.PlacedTargetRemainingTranslationRange]
        placeLocationInfos=None,  # type: Optional[list[types.PlaceLocationInfo]]
        planningSchedulingMode=None,  # type: Optional[Literal['ParallelPlanRequests', 'SinglePlanRequestPriority']]
        planningSmallestObjectSizeForCollision=None,  # type: Optional[float]
        planToDestMode=None,  # type: Optional[Literal['None', 'All']]
        predictDetectionInfo=None,  # type: Optional[types.PredictDetectionInfo]
        postCycleExecution=None,  # type: Optional[types.PostCycleExecutionConfigurations]
        putBackParameters=None,  # type: Optional[types.PutBackParameters]
        randomBoxInfo=None,  # type: Optional[types.RandomBoxInfo]
        recoverySpeedMult=None,  # type: Optional[float]
        rejectGraspOverlappingNonPickable=None,  # type: Optional[Literal['disabled', 'prioritize', 'always']]
        restrictLabelOrientationInfo=None,  # type: Optional[types.RestrictLabelOrientationInfo]
        robotCycleStartPosition=None,  # type: Optional[list[float]]
        robotFinishPosition=None,  # type: Optional[list[float]]
        robotRecoveryPosition=None,  # type: Optional[list[float]]
        robotRecoveryDepartAccel=None,  # type: Optional[float]
        robotRecoveryDepartSpeed=None,  # type: Optional[float]
        robotRecoveryDepartOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        robotRecoveryDepartOffsetInTool=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveEnvState=None,  # type: Optional[bool]
        saveEnvStateOnUnexpectedFinish=None,  # type: Optional[bool]
        saveGrabRecoveryScene=None,  # type: Optional[bool]
        savePlanningDiagnosticData=None,  # type: Optional[bool]
        saveRecoveryScene=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        saveOnPlanFailure=None,  # type: Optional[bool]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        saveVerificationScene=None,  # type: Optional[bool]
        saveWhenSlowPlanningDuration=None,  # type: Optional[float]
        timeOutForFailedParts=None,  # type: Optional[float]
        useKZFilter=None,  # type: Optional[bool]
        singularityEpsilon=None,  # type: Optional[float]
        skipCollidingDestsInfo=None,  # type: Optional[types.SkipCollidingDestsInfo]
        skipLastImageCheckWhenNoMoreDestForDynamicGoals=None,  # type: Optional[bool]
        skipTrajectoryPlanning=None,  # type: Optional[bool]
        splitterSheetInfo=None,  # type: Optional[types.SplitterSheetInfoSchema]
        smootherParameters=None,  # type: Optional[types.SmoothingParameters]
        sourceDynamicGoalsGeneratorParametersOverwrite=None,  # type: Optional[types.StartPickAndPlaceThreadParametersSourceDynamicGoalsGeneratorParametersOverwrite]
        sourcecameranames=None,  # type: Optional[list[str]]
        sourcecontainername=None,  # type: Optional[str]
        sourceDestTargetOrientationPenalty=None,  # type: Optional[float]
        strictPickOrdering=None,  # type: Optional[types.StrictPickOrdering]
        targetdestikthresh=None,  # type: Optional[float]
        targetenvclearance=None,  # type: Optional[float]
        targetGraspTimeLimit=None,  # type: Optional[float]
        targetIsRandomBox=None,  # type: Optional[bool]
        targetMinBottomPaddingForInitialTransfer=None,  # type: Optional[float]
        targetMinSafetyHeightForInitialTransfer=None,  # type: Optional[float]
        targetPaddingForGrasping=None,  # type: Optional[float]
        targetPlaceIkParamName=None,  # type: Optional[str]
        targetPlaceTranslationOffset=None,  # type: Optional[tuple[float, float, float]]
        targetPlaceTranslationOffsetInDest=None,  # type: Optional[bool]
        targetPostActionAfterPick=None,  # type: Optional[Literal['setUnpickable', 'moveDownSetUnpickable', 'delete', 'deleteButConsiderForApproachMove', 'deleteNotifyBridge']]
        targetPriorityMultipliers=None,  # type: Optional[types.TargetPriorityMultipliers]
        targetStackDestHeightPenaltyMult=None,  # type: Optional[float]
        targetPrioritySuctionForceMult=None,  # type: Optional[float]
        targetPriorityTransferSpeedMult=None,  # type: Optional[float]
        targeturi=None,  # type: Optional[str]
        targetThicknessThreshForTrapping=None,  # type: Optional[float]
        toolMaxRotationBetweenPickAndPlace=None,  # type: Optional[float]
        targetRotationConstraintParameters=None,  # type: Optional[types.TargetRotationConstraintParameters]
        targetLabelAlignmentParameters=None,  # type: Optional[types.TargetLabelAlignmentParameters]
        treatAsSquareTargetExtentsThreshold=None,  # type: Optional[float]
        toolPosConstraintPaddingXYZ=None,  # type: Optional[tuple[float, float, float]]
        toolposes=None,  # type: Optional[types.StartPickAndPlaceThreadParametersToolposes]
        toolSpeedAccelInfo=None,  # type: Optional[types.ToolSpeedAccelInfo]
        toolSpeedAccelOptions=None,  # type: Optional[int]
        torqueDistThresh=None,  # type: Optional[float]
        transferSpeedPostMult=None,  # type: Optional[float]
        minAcceptedTransferSpeedMult=None,  # type: Optional[float]
        minTransferSpeedMult=None,  # type: Optional[float]
        transferSpeedMultPerWeight=None,  # type: Optional[list[types.TransferSpeedMultPerWeight]]
        transferTrajectoryCostMultipliers=None,  # type: Optional[types.TransferTrajectoryCostMultipliers]
        transferTrajectoryCostFnDiscretizationStep=None,  # type: Optional[float]
        updateHeightWithMeasuredValue=None,  # type: Optional[bool]
        updateMassWithMeasuredValue=None,  # type: Optional[bool]
        unitMass=None,  # type: Optional[float]
        useBarcodeUnpickableRegions=None,  # type: Optional[bool]
        useDecreaseTargetEnvClearanceBigObjects=None,  # type: Optional[bool]
        useDetectedFace=None,  # type: Optional[bool]
        useDestGoalsInCollision=None,  # type: Optional[bool]
        useDropInSourceDests=None,  # type: Optional[bool]
        useDynamicGoals=None,  # type: Optional[bool]
        useExecutionQueueing=None,  # type: Optional[bool]
        useFailThresholds=None,  # type: Optional[bool]
        useLocationState=None,  # type: Optional[bool]
        waitDynamicGoalPointCloudTimeout=None,  # type: Optional[float]
        validatePlacedAllPickable=None,  # type: Optional[bool]
        waitForBetterCostTime=None,  # type: Optional[float]
        waitForSupplyTimeout=None,  # type: Optional[float]
        waitForDetectionAfterInitialPick=None,  # type: Optional[bool]
        waitForDestProhibitedAtGrasp=None,  # type: Optional[bool]
        waitForSourceProhibitedAtDest=None,  # type: Optional[bool]
        waitForStateTrigger=None,  # type: Optional[str]
        waitForLocationUnprohibited=None,  # type: Optional[bool]
        waitUpdateStampTimeout=None,  # type: Optional[float]
        ykkControlInfo=None,  # type: Optional[types.YKKControlInfo]
        containername=None,  # type: Optional[str]
        controllerclientparameters=None,  # type: Optional[types.StartPickAndPlaceThreadParametersControllerclientparameters]
        cycleIndex=None,  # type: Optional[str]
        cycleStartUseToolPose=False,  # type: bool
        destGoals=None,  # type: Optional[types.DestGoals]
        destcontainernames=None,  # type: Optional[list[str]]
        destSensorSelectionInfos=None,  # type: Optional[list[types.StartPickAndPlaceThreadParametersDestSensorSelectionInfosArrayElement]]
        detectionInfos=None,  # type: Optional[list[types.StartPickAndPlaceThreadParametersDetectionInfosArrayElement]]
        forceMoveToFinish=None,  # type: Optional[bool]
        forceStartRobotPositionConfigurationName=None,  # type: Optional[str]
        ignoreStartPosition=None,  # type: Optional[bool]
        initiallyDisableRobotBridge=None,  # type: Optional[bool]
        inputPartIndex=None,  # type: Optional[int]
        itlParameters=None,  # type: Optional[types.StartPickAndPlaceThreadParametersItlParameters]
        locationCollisionInfos=None,  # type: Optional[list[types.StartPickAndPlaceThreadParametersLocationCollisionInfosArrayElement]]
        pickContainerHasOnlyOnePart=False,  # type: bool
        placedTargetPrefixes=None,  # type: Optional[list[str]]
        registrationInfo=None,  # type: Optional[types.StartPickAndPlaceThreadParametersRegistrationInfo]
        sourcecontainernames=None,  # type: Optional[list[str]]
        sourceSensorSelectionInfos=None,  # type: Optional[list[types.StartPickAndPlaceThreadParametersSourceSensorSelectionInfosArrayElement]]
        targetname=None,  # type: Optional[str]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Start a background loop to continuously pick up objects with the targetnamepattern and place them down at the goals. The loop will check new objects arriving in and move the robot as soon as it finds a feasible grasp. The thread can be quit with StopPickPlaceThread.

        Args:
            goaltype: (Default: None)
            goals: (Default: None)
            targetnamepattern: This supports regular expressions (i.e. detected_\d+) (Default: None)
            approachoffset: mm. The distance between the approach point P1 where the linear (slow) approach motion starts and the grasp point P2 where the robot stops and the gripper opens/closes to grasp the target item. 

            It is appropriate to set the value at least as deep as the gripper's usual grasp depth (how far it moves into or towards the item to grasp it).

            Setting a low value can cause collisions with noisy detections in the bin, and/or slower planning, as the final motions towards the grasp pose can be very constrained and sampling in joint space is inefficient. By default, the motion to the approach point is sampled in joint space.

            Common values are 30-100 mm. Setting a value that is too high can cause pick candidates to be discarded, as the approach motion may collide with e.g. container walls or be out of the robot's reach. (Default: 30)
            departoffsetdir: Departure offset direction. mm (x,y,z) (Default: [0, 0, 50])
            destdepartoffsetdir: Departure offset direction. mm (x,y,z) (Default: [0, 0, 30])
            deletetarget: Whether to delete target after pick and place is done. (Default: 0)
            debuglevel: Limits the amount of logs that are written. High log levels should only be active when a problem is being investigated, as excessive logging can decrease performance. Log levels lower than 3 are not recommended.

            - Level 0: Only FATAL logs
            - Level 1: Include ERROR logs
            - Level 2: Include WARNING logs
            - Level 3: Include INFO logs
            - Level 4: Include DEBUG logs
            - Level 5: Include VERBOSE logs
             (Default: 4)
            movetodestination: If 1, then the robot will place the part in the destination after picking it. If false, then robot will stop the cycle after picking. 

            This is mainly used for testing, and should be set to 1 for production.
             (Default: 1)
            worksteplength: **deprecated** Planning parameter. (Default: None)
            regionname: (Default: None)
            envclearance: (Default: None)
            toolname: (Default: None)
            robotspeed: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            allTargetsDifferentUri: Whether the binpickingmodule should treat all targets as different. (Default: None)
            absMaxPlanningTimeToWait: seconds. Valid if > 0. If a valid pick-place plan result has been computed and the system has been waiting for more than the specified time, then the best plan result is returned. If no valid plan result is available yet, the system will continue planning until all the candidate grasp ikparams, dest ikparams, and their combinations have been checked. If no valid plan result is computed and all the candidate IKParams are exhausted, then the system will return error codes such as `novaliddgrasp`, `novaliddest`, or `novalidgraspgoalpair`.

            There are other planning time limits like `smoothingTimeLimit` in SmootherParameters, `maxPlanningTime` in PathPlannerParameters, `iktimelimit` for finding IKSolutions, etc. that affect the time spent on parts of the pick-place planning process. (Default: None)
            addGraspGoalPairWorker: If True, adds a planning worker thread that works only on grasp-goal pair generation. (Default: None)
            alwaysPlanOutOfOcclusion: If true, plans a motion out of camera occlusion when a pick failed or robot had to stop and retry a pick (i.e. piece lost). (Default: None)
            approachCurrentExceedThresholds: Absolute current thresholds that should not exceed by joints (Default: None)
            approachCurrentExceedThresholdsDelta: Check if any of the currents exceed the difference between the start current values (at torqueCheckStartTime) and the current ones. If they do, then will return torque error. Do the check after torqueCheckStartTime. (Default: None)
            approachForceTorqueExceedThresholds: Absolute force and torque thresholds that should not be exceeded. Unit is N for force (first 3 elements FX, FY, FZ), Nm for torque (last 3 elements MX, MY, MZ). A negative value means that the component is disabled. (Default: None)
            approachForceTorqueExceedThresholdsDelta: Check if any of the force and torque exceed the difference between the values at start (at torqueCheckStartTime) and the current ones. If they do, then will return torque error. Unit is N for force, Nm for torque. A negative value means that the component is disabled. The check is performed after torqueCheckStartTime. (Default: None)
            approachOffsetFromWalls: mm. When the tool is close to container walls at a grasp position, the approach will be offset from the walls. (Default: None)
            atStartPlanDynamicContentsNames: An axis-aligned bounding box (AABB) will be created based on specified bodies. The collision of AABB is checked only during start plan. AABB aligns to world frame. AABB is created not to collide with AABB of manipulator's geometry but its clearance against manipulator is controlled to be close in order to avoid collision against dynamic contents (Default: None)
            automaticToolPosConstraintNotGrabbing: If true, will clamp the position of the tool when not grabbing an item when approaching the source location or when moving to finish. (Default: None)
            automaticToolPosConstraintWhenGrabbing: If true, will clamp the position of the tool to a range between the start and goal of the transfer plan. This will force the tool not to move too far up or down while performing the transfer. Use 'toolPosConstraintPaddingXYZ' to pad the XYZ of each dimension for the tool. (Default: None)
            barcodeScanningInfo: Describes the barcode scanners and how the robot computes the barcode scanning motion. (Default: None)
            binpickingDebugMode: Can be used to debug the pick and place grasps and dest goals.

            - **BDM_CheckAllGrasps=0**: Disabled.
            - **BDM_CheckAllGrasps=1**: If set, then will not early prune grasps so that user can tell which grasps really did fail. nTargetGraspLimit, nGraspTimeLimit, nMaxGraspIkSolutions, nMaxGraspIkSolutionsPerGrasp are set to 0.
            - **BDM_CheckAllDests=2**: If set, then will not early prune dests so that user can tell which dests really did fail. nMaxDestIkSolutions, fMaxAcceptedDestPlanTrajTime are set to 0.

            By default, this debug mode is disabled. (Default: None)
            bodyNameToAvoidFinalCollision: If specified, during the final trajectory, enable collision avoidance with this body.

            During the regular pick cycle, collision checking for some bodies such as wiring tubes may be turned off. In the final plan, when the robot moves away from the camera, it may be necessary to enable this body for collision checking, e.g. to avoid camera occlusion. (Default: None)
            bottomScanPlacementInfo: Parameters for placing target with uncertain height to conveyor by considering distance measured by the sensors. (Default: None)
            cameraPlanningInfos: A list of planning infos for cameras that are attached to the robot. (Default: None)
            cameraOcclusionApplyGrabbedState: If True, then when doing occlusion checking, use the correct grabbed object to do occlusion checking. If object is small and completely covered by gripper, this can be set to False to get performance boost. (Default: None)
            cameraOcclusionOffset: mm, offset is used in detection of camera occlusion by the robot (Default: None)
            cameraOcclusionPaddingTime: s, time offset to expand camera start and end camera capture times for occlusion detection. (Default: None)
            cameraOcclusionPaddingTimeStart: s, time offset to expand camera start of camera capture times for occlusion detection. This is useful when uncertainty of occluding time has asymmetry on start and end. This has priority over cameraOcclusionPaddingTime (Default: None)
            cameraOcclusionPaddingTimeEnd: s, time offset to expand camera end of camera capture times for occlusion detection. This is useful when uncertainty of occluding time has asymmetry on start and end. This has priority over cameraOcclusionPaddingTime (Default: None)
            cameraOcclusionUseLinkVisibility: If True, then when doing occlusion checking, consider all the visible but disabled links in the check. This can be cables and other movable objects. If invisible objects do not affect occlusion status, set this to False to get performance boost. (Default: None)
            canPlaceInSourceOnRecover: If True, then allow robot to place back the part in the source when robot has to recover. (Default: None)
            canPlaceInSourceOnBarcodeScanFail: If True, then allow robot to place back the part in the source when barcode scanning fails. (Default: None)
            canSetIsPickable: If set to True, then allows planning to set isPickable flags to False to any targets it picks up and knows that it should not pick up again unless they get updated. If False, does not allow changing the isPickable flag. Set to True by default. (Default: None)
            checkCameraOcclusionAtMidDest: If True, then checks occlusion with the source cameras at mid dest. (Default: None)
            checkCollisionAtDestNames: Enables specified bodies for collision detection at the destination placement. The body names can be e.g. an external IO region at the destination. (Default: None)
            checkDestContainerEmptyOnArrivalNames: If the `checkPlaceContainerEmptyOnArrival` flag is True, then the destination container names in this list will be checked to make sure the place container is empty upon arrival. A non-empty place container will trigger the error code `FinishedPlaceContainerNotEmpty`. (Default: None)
            checkExpectedDetectedHeightThreshold: The threshold for the expected detected height of the top surface of the target workpiece to determine height check failures. If `checkExpectedDetectedHeight` is set, the threshold is compared against it. If not, the threshold is compared against N*`part_height` where N is the number of layers. (Default: None)
            checkForEndEffectorLowerThanGraspDist: mm. Distance threshold for the max distance of "end effector translation"-"tool translation" along the vector oriented toward the container opening (usually +z). (Default: None)
            checkObstacleNames: (Default: None)
            checkPickContainerEmptyOnFinish: If True, then check to make sure the pick container is empty when the order cycle finishes. If it is not empty, the cycle will finish with error code 'FinishedPickContainerNotEmpty'.

            If this is set to True, it overrides computePickContainerEmptyOnFinish. (Default: None)
            checkPlaceContainerEmptyOnArrival: If True, then check to make sure the place container is empty upon arrival. Non-empty place container will trigger the error code FinishedPlaceContainerNotEmpty. (Default: None)
            clearanceTopDestContainer: mm. Defines how far away the tool should be from the destination container top when starting to approach or finishing to depart from the container. If set to 0, has no effect. (Default: None)
            clearanceTopSourceContainer: mm. Defines how far away the tool should be from the source container top when starting to approach or finishing to depart from the container. If set to 0, has no effect. (Default: None)
            computePickContainerEmptyOnFinish: If True, then computes (from vision data) if the pick container is empty or not after the final pick from the pick container. This will take an extra snapshot with the camera, so can reduce cycle time and slow down the move-out of the pick container. The result ('isContainerEmpty') is added to the log and the robot will continue.

            To stop with an error code when the container is non-empty after picking, set checkPickContainerEmptyOnFinish to True. It overrides this parameter.  (Default: None)
            computePickContainerDamagedOnFinish: If True, then computes (from vision data) if the pick container is damaged or not after the final pick from the pick container. This will take an extra snapshot with the camera, so can reduce cycle time and slow down the move-out of the pick container. The result ('isContainerDamaged') is added to the log and the robot will continue. (Default: None)
            constraintToolInfo: Constrain a direction on the tool to be within a certain angle with respect to a global direction. (Default: None)
            constraintToolPosToSource: If True, then the tool position jittering is constrained to the inside of the source container. For containers with walls, this is vital to ensure the tool does not attempt to jitter outside of the container walls, which can cause accidents.

            When using virtual containers which do not have walls, e.g. a pallet, this parameter should be set to 'False' to avoid placing unnecessary constraints on planning.
             (Default: None)
            containerEmptyWaitTime: Time to wait after vision reports that the source container is empty (the `isContainerEmpty` signal is received). By default, this is 0 and the robot finishes when the container is empty.

            During this wait time, the system waits for new detections. This can be used to run detection additionally on the source container, for example when detection does not reliably detect pieces in the container. (Default: None)
            cycledests: Max number of times to repeat the destikparamnames, 0 means infinite repeat (Default: None)
            deleteDynamicObjectsAfterFinish: If True, will delete all dynamic objects after finishing cycle (for cleaning up) (Default: None)
            deleteTargetWhenPlacedInDest: If True, then delete the target as soon as it is placed in the destination. This simulates machines or drop off locations which take the part as soon as the robot releases them. (Default: None)
            deleteTargetFromSourceContainer: 0: do not delete target after it is picked up until the next vision detection update changes the targets. 1: delete the target after it is picked up. Setting this to 1 allows the robot to pick up previously overlapped parts without waiting for a vision update, but it is also more dangerous since there can be parts below the picked up target that were previously getting occluded, and removing the picked up target might wrongly tell the robot that the below region is free space. If 2, then only consider the delete targets during grabbing the part, but still consider them during approach. (Default: None)
            destikparamnames: Names of the intermediate destination IK parameters to which the robot will go and release the part. Should be in the format instobjectname/ikparamname (Default: None)
            intermediateCycles: List of information specifying conditions for when to do an intermediate cycle for target re-grabbing. Conditions for intermediate cycle can very, which is why there is a list of them. (Default: None)
            deleteTargetsOnPieceLost: If True, when the piece is lost and robot has to recover, will remove the all detected targets from the scene and wait for new ones to come. (Default: None)
            deleteTargetsOnRecovery: If True, during any recovery because of a failure to pick or move the robot, will remove the all detected targets from the scene and wait for new ones to come. (Default: None)
            deleteTargetsEveryCycleSlowMode: If True, will delete the current targets after every pick and force re-detection of the parts. Used when robot has to stop and rescan every frame. (Default: None)
            destBarcodeScanningInfo: Describes the barcode scanning on the dest (Default: None)
            destBarcodeScanningInfoPerContainer: Set of destBarcodeScanningInfo per container (Default: None)
            destTargetAABBAlignIkParameters: Parameters used for computing target placement when 'destCoordType' is 'targetAABBAlignIk' (Default: None)
            destTargetAnyBottomFaceRotationParameters: Parameters used when placing a part at the destination when 'destCoordType' is targetAnyBottomFace. (Default: None)
            destTargetCornerParameters: Parameters used when placing targets at the destination with 'destCoordType' being 'targetCorner'. (Default: None)
            destTargetStackParameters: Parameters used when placing targets at the destination with 'destCoordType' being 'targetStack'. (Default: None)
            dynamicGoalsParameters: If 'useDynamicGoals' is True, then will be dynamically generating goals based how to call on the packing algorithms. Internally, the packing algorithms parameters are managed by packFormation profiles.
             (Default: None)
            ensurePickVisibilityAtPlace: Ensure pick container visibility when the robot is at place location. The types are:
                        - **required**: always reject occlusion dest solutions.
                        - **preferred**: reject occlusion dest solutions at the first planning round. If planning fails in the previous round with novaliddest or novalidgraspdestpair, accept occlusion dest solutions.
                        - **disabled**: always accept occlusion dest solutions.
                         (Default: None)
            graspApproachInfos: Information defining translation offsets, rotation offsets, and other related parameters the tool to approach to a grasp. If the approach offset from the grasp should be used, please define "segments" as follows.
            - The "segments" list contains only one element.
            - The element does not contain "translationOffset".
            - The element contains "transSpeed" and "transAccel". (Default: None)
            graspApproachInfosPerURI: (Default: None)
            graspDepartInfos: Information defining translation offsets, rotation offsets, and other related parameters for the tool to depart from a grasp after grasping a target. (Default: None)
            graspDepartAboveNearbyObstacles: If True, then the robot extends the motion along the first graspDepartInfo segment such that the bottom of the grabbed target is above any of the nearby obstacles that were within neighTargetThresh.

            The motion is always in the direction of the first graspDepartInfo segment and is at least the length of graspDepartInfo segment.

            By default, the target buffer between the bottom of the grabbed target and the top of the surrounding obstacles is 5 cm along the first graspDepartInfo segment. (Default: None)
            graspDepartAboveNearbyObstaclesMaxDist: mm. Limits the maximum distance that graspDepartAboveNearbyObstacles may add to the first graspDepartInfo segment.

            If graspDepartAboveNearbyObstacles is True, then the departing motion extends motion along the first graspDepartInfo segment such that the bottom of the grabbed target is above any of the nearby obstacles that were within neighTargetThresh.

            If this parameter is too small, it may cause the bottom of the grabbed target to be closer to nearby obstacles.

            If graspDepartAboveNearbyObstacles is False, this parameter has no effect. (Default: None)
            departOffsetFromWalls: mm. When the part is close to the container walls, this is the buffer that will be added during the linear depart motion, after the part is picked.


            The direction and angle of the motion depend on the first graspDepartInfo segment and other parameters. (Default: None)
            destApproachCurrentExceedThresholds: Absolute current thresholds that should not exceed by joints during destination approach (Default: None)
            destApproachCurrentExceedThresholdsDelta: Check if any of the currents exceed the difference between the start current thresholds (at torqueCheckStartTime) and the current ones. If they do, then will return torque error. Do the check after torqueCheckStartTime. (Default: None)
            destApproachForceTorqueExceedThresholds: Absolute force and torque thresholds that should not exceed during destination approach. Unit is N for force, Nm for torque. A negative value means the component is disabled. (Default: None)
            destApproachForceTorqueExceedThresholdsDelta: Check if any of the force and torque exceed the difference between the values at start (at torqueCheckStartTime) and the current ones. If they do, then will return torque error. Unit is N for force, Nm for torque. A negative value means the component is disabled. Do the check after torqueCheckStartTime. (Default: None)
            destApproachClearContainerTop: If True, will ensure that the first destination approach starts from outside of the destination container. This can improve the likelihood of planning success under certain conditions, e.g. when a container is cluttered.

            Setting this parameter forces the robot to start approaching the destination from outside the container. This can increase the quality of the `transferToDest` plan (shorter planning time, shorter execution time, smoother appearance, etc.)

            Example 1: If the destination container is cluttered, the regular approach settings in `destApproachInfos` may place the `destApproachStart` point inside the container. As joint-space path planning is used for the `transferToDest` motion which ends at the `destApproachStart` point, the clutter can make motion planning difficult. In this case, turning this parameter on would make the `transferToDest` motion easier to plan.

            Example 2: In the picture, the robot places a container in a shelf. It is better to enable this parameter and start to approach the destination from outside of the shelf container, as the clearance inside the shelf is too small for joint-space path planning. (Default: None)
            destApproachInfos: Information defining translation offsets, rotation offsets, and other related parameters for the tool to approach to a destination while grasping a target. (Default: None)
            destApproachAccelDecelScaleMultOnTargetMass: This array is used to compute scaling multiplier that scales dest approach acceleration and deceleration from target mass. Computed scaling multiplier will be saturated by 1.0. This is useful when mass validation is enabled and there is difficulty on computing accurate inertial force compensation. (Default: None)
            destApproachAccelDecelMultOnTilt: Multiplier for tool acceleration/deceleration when approaching to the destination with tilt (Default: None)
            destApproachSpeedMultOnTilt: Multiplier for tool speed when approaching the destination with tilt (Default: None)
            destApproachSpeedMultOnUnknownSize: Destination approach speed multiplier for a target of unknown size. This is used the first time that a box of a certain type (SKU) is picked and its size is unknown. The size of the box is measured when it is first picked (auto-registration) and used for subsequent picks.

            Also see:

            - decelSpeedAfterIOMatches
            - skipDecelOnKnownHeight
             (Default: None)
            destcontainername: (Default: None)
            destcoordtype: Coordinate system type of the destination. can be one of:
            - 'tool' specifying the current tool,
            - 'toolzntarget' specifying the point of the tool projected along the -z axis until the target boundary,
            - 'target' specifying the original target coordinate system, or
            - 'targetbottom' that offsets the target so its grasped bottom aligned with the ikparam, or
            - 'targetAnyBottomFace' specifying that robot should put the target on its bottom face regardless of which grasp it makes. The center is aligned with the center of the target bounding box.
            - 'targetAnyBottomFaceAlignedX' specifying that robot should put the target on its bottom face regardless of which grasp it makes such that the X axis of the object is aligned with X axis of the goal, or
            - 'targetAnyBottomFaceAlignedY' specifying that robot should put the target on its bottom face regardless of which grasp it makes such that the Y axis of the object is aligned with X axis of the goal, or
            - 'targetAnyBottomFaceXIsLongAxis' specifying that robot should put the target on its bottom face regardless of how it is grasped and to align the longest side of the target with the X-axis of the goal. Will align so that the target X axis is always toward the X of the ikparam. Use 'destTargetAnyBottomFaceRotationParameter.facePlaceEdge' to control which corner to align the box with.
            - 'targetCorner' - Align any corner of the target with the 6D coordinate system.
            - 'targetStack' - stack targets on top of previously placed targets. Prioritize placement on lower stacks.
            - 'targetAnyBottomBarCodeFace' - specifying that robot should put the target on its bottom face regardless of how it is grasped and to align the registered known barcode faces to the barcode reader. if there are no known barcode faces, does the same as targetAnyBottomFaceXIsLongAxis. Use 'destTargetAnyBottomFaceRotationParameter.facePlaceEdge' to control which corner to align the box with.
            - 'targetAABBAlignIk' - Align target's AABB with the dest ikparam
             (Default: None)
            destDepartFutureCycleMerge: If True, dest depart trajectory will be merged with approach trajectory of the next cycle if the next plan is already available. (Default: None)
            destDepartInfos: Information defining translation offsets, rotation offsets, and other related parameters for the tool to depart from the destination after placing a target. (Default: None)
            destDepartSpeedMultOnUnknownSize: destination depart speed multiplier on unknown-sized target. has to be greater than 0.01, less than or equal to 1.0 (Default: None)
            destDepartClearContainerTop: If True, will ensure that the last destination depart finishes outside of the destination container. This can improve the likelihood of planningsuccess under certain conditions, e.g. when a container is cluttered.

            Setting this parameter forces the robot to finish departing the destination outside the container. This can increase the quality of the `Finish` plan (shorter planning time, shorter execution time, smoother appearance, etc.)

            Example 1: If the destination container is cluttered, the regular depart settings in `destDepartInfos` may place the `destDepartEnd` point inside the container. As joint-space path planning is used for the `Finish` motion which starts at the `destApproachEnd` point, the clutter can make motion planning difficult. In this case, turning this parameter on would make the `FinishPlan` motion easier to plan.

            Example 2: In the picture, the robot places a container in a shelf. It is better to enable this parameter and finish to depart the destination outside of the shelf container, as the clearance inside the shelf is too small for joint-space path planning. (Default: None)
            destFilterByTargetOrientationThresh: Enabled if `destcoordtype` is 'target'. The angle is used to filter destination goals such that they only allow targets in the source container that are oriented around the container's up axis. The allowed rotation angle between the source and destination target poses (normalized by the in-plane rotation, around the destination container's up axis), is limited by this value. If less than 0, then invalid. (Default: None)
            destFilterByTargetOrientationThreshPerMass: Filter destinations based on target orientation per mass. (Default: None)
            destPriorityLabelDirectionValue: The base priority value to apply to destination goals with a good label position. IK Parameters with a bad label position will have lower priority.

            This parameter is added to the priority of the destination IKparam generated by the dynamic goal generator to prioritize placement of label (barcode, QR code, etc.) in particular direction in the container.

            There are different modes to orient the label:
            - prioritize placement of the label to face outside of the container
            - prioritize label orientation in world or container coordinate system. A corresponding directional vector of preferred orientation needs to be provided with labelerDirection and restrictLabelOrientationInfo parameter. 

            There are 3 possible results of label alignment:
            - If label is aligned as requested, then `destPriorityLableDirectionValue` (positive) will be added to DGG priorities to prioritize such an orientation.
            - If system cannot determine a good label orientation, then `destPriorityLabelDirectionValue` = 0, will be applied to DGG priorities to not consider it.
            - If label placed is in the opposite direction of the preferred label position, `destPriorityLabelDirectionValue`(negative), will be applied to DGG priorities, to penalize (subtract priority) such a label orientation. (Default: None)
            destTargetValidationJitterDist: mm, if a target at the destination is in collision, then can jitter by this distance so that a new collision-free goal could be used instead. This is used when a destination point cloud is used to pack items. (Default: None)
            detectionResultsMaxCacheTime: If > 0, then the detector come give old cached detection results back even though the sensor time stamp is updated. So this is the max time between a sensor timestamp and the result returned for that timestamp. Most detectors have this as 0 since they take only one sensor snapshot. Some detectors that need to do sensor fusion can be giving out old cached results. (Default: None)
            discardTargetIfJitterFails: Specifies the number of times jitter has to fail before discarding the entire target. (Default: None)
            discardTargetIfMergeAfterJitterFails: Specifies the number of times merging after jitter has to fail before discarding the entire target. (Default: None)
            disablePlacedTargetsInPickLocationWhenPlanning: If True, then any bodies that have been placed before to that location will be disabled and ignored during planning. This is used when a sensor is re-sensing the bodies and therefore will produce its own detected_ versions of the bodies. (Default: None)
            disableTargetCheckOnGraspApproach: If True, then disable the target body for collision checking during grasp approach. By default, this is true, since the tool may need to slightly touch the target when it ends GraspApproach and starts GraspDepart. (Default: None)
            disableUnchuck: If True, then gripper unchucking will be disabled before finishing the cycle. By default this is False. It can be used to enter ITL programs without  (Default: None)
            detectionTriggerMode: If 'AutoOnChange' (default), then snaps whenever the camera is unoccluded and the source container has changed since the last capture.

            If 'WaitTrigger', then the detector waits for `triggerDetectionCaptureInfo` to be published by planning in order to trigger the detector, otherwise it will not capture.

            If 'Continuous', then continuously triggering the detection regardless of occlusion. (Default: None)
            distanceMeasurementInfo: Parameters for measuring the height of a target object with a 1D distance sensor. Setting up these parameters will send timed IO values as the robot trajectory is executed. (Default: None)
            doAccurateGraspDepart: If False, the robot may try to shortcut the departing trajectory after grabbing a part to improve performance. If the clearance with other parts at grasping points is tight, this should be set to True. (Default: None)
            doAutoRecoveryOnPieceLost: This parameter is only considered if isStopOnPieceLost is True. If this parameter is True and PieceLost occurs inside the source container, recovery is attempted. If PieceLost occurs outside the source container, the robot stops without recovering (since it is unclear where the piece fell). (Default: None)
            doAutoRecoveryOnRobotExecutionError: If True, will try to recover after a robot execution error occurs. (Default: None)
            doSimultaneousGripperPreshapingDuringApproach: If True, gripper preshaping is performed simultaneously with the robot's Approach movement. If False, gripper preshaping is performed before starting the robot's Approach movement.

            Preshaping moves the gripper joints into the position required to perform a grasp.

            This parameter is applicable only if the system determines that gripper preshaping during robot motion can be allowed. The binpicking module will first check the scene environment for the feasibility of gripper motion along with robot motion. With certain robot configurations, the space may be too tight for simultaneous gripper preshaping. The preshaping motion must be feasible and the gripper controller must support preshaping by allowing asynchronous IO changes.

            For example, the L-shaped gripper computes the gripper motion automatically based on the desired goal position, so it is possible to use this parameter to perform simultaneous gripper preshaping during the robot's grasp approach motion. (Default: None)
            dropInDestInfo: Parameters to define how to drop object to the destination. (Default: None)
            dropInDestInfoPerContainer: Set of dropInDestInfo per container (Default: None)
            dropOffParameters: Parameters to drop off. (Default: None)
            dropOffsetFromCollision: For destinations with isDropInSource set to True, try to lower the drop point so that it is [dropOffsetFromCollision] away from the top of the parts in the pick container. (Default: None)
            dropTargetInSourceContainerBoxMult: Multiplier for the target AABB to determine if safely dropped in source container. the higher the value, the more safety there is in accidentally thinking that the part was dropped inside the container. If robot is getting FinishedDropTargetFailure while object is still in container, then reduce this value. (Default: None)
            dropTargetInDestContainerBoxMult: Multiplier for the target AABB to determine if safely dropped in dest container. the higher the value, the more safety there is in accidentally thinking that the part was dropped inside the container. (Default: None)
            dropTargetInDestContainerZSafetyMult: If target is inside the XY dest constraints, this controls what is the max height of the target (in terms of its multiples) where it is ok to drop the target because of torque limits/piece lost errors. in other words, the amount of z extents that could be outside of the dest and still success will be declared. The lower the value, the more conservative it is. (Default: None)
            dropTargetMaxDistanceThresold: Maximum distance from the grasp trajectory start point to determine if the part safely dropped inside of the source container. If 'isStopOnPieceLost' is False, then recover regardless this parameter; if 'isStopOnPieceLost' is True and the part safely dropped then try to recover from PieceLost error. Assumes the part is lost if this is negative. (Default: None)
            dropTargetMaxDistanceXYThreshold: Maximum XY distance from the grasp trajectory start point to determine if the part safely dropped inside of the source container. If 'isStopOnPieceLost' is False, then recover regardless this parameter; if 'isStopOnPieceLost' is True and the part safely dropped then try to recover from PieceLost error. (Default: None)
            enableBodyNamesOnCameraPlan: Enable specified bodies when computing IK solutions for camera manipulation (eye-in-hand), and when planning for trajectories to camera poses.

            For setups with cameras mounted on the robot ("eye-in-hand"), the robot needs to move the camera above the source container to record an image before picking. During the motion to this pose, the body names in this parameter will be enabled for collision avoidance, both when IK solutions are computed to the camera poses and when planning the trajectories to the poses. (Default: None)
            enableBodyNamesOnDestPlan: Enable specified bodies for collision avoidance during motion planning to the destination.

            For example, in picking applications with multiple source or destination containers, the containers not involved in a particular pick-place cycle should be enabled for collision avoidance during the "dest plan" motion. (Default: None)
            enableBodyNamesOnStartPlan: Enable specified bodies for collision avoidance during start plan motion planning.

            For example, in picking applications with multiple source or destination containers, the containers not involved in a particular pick-place cycle should be enabled for collision avoidance during the "start plan" motion. (Default: None)
            encoderConvergenceSpeedThresh: deg/sec. The robot is assumed to have reached the commanded position when the sum of absolute values of actual joint velocities is lower than this threshold. The smaller the value, the more accurate the robot will be, but the process will be slower. (Default: None)
            executionConnectingTrajDecelMult: The parameter is used to control how fast the robot decelerates when pausing due to an interlock or PieceLost problem. (Default: None)
            executionConnectingTrajReverseMult: The parameter is used to control how fast the robot moves backward and then stops when reversing due to torque limit error or position tracking error. (Default: None)
            executionReverseRecoveryDistance: Recovery distance the robot tool should move in reverse of the trajectory from the detected collision/error position. (Default: None)
            executionEnvClearanceApproachConcatenate: The environment clearance used for approach trajectory concatenation with merging. Set to 0.0 to speed up concatenation, but can be more dangerous. (Default: None)
            executionMaxConcatenateSearchTime: The maximum search time in common.ConcatenateTrajectoriesWithMerging. (Default: None)
            executionConcatenateSearchDeltaTime: The deltatime in common.ConcatenateTrajectoriesWithMerging. If 0 - the concatenation is disabled. (Default: None)
            executionFilterFactor: the parameter is used to make robot trajectory while grabbing not grabbing  a target smooth. Default is 1.0, a smaller value makes the trajectory smoother, but the trajectory is rejected more often and can possibly collide with obstacles. (Default: None)
            executionFilterFactorWhenGrabbing: the parameter is used to make robot trajectory while grabbing a target smooth. Default is 0.4, a smaller value makes the trajectory smoother, but the trajectory is rejected more often and can possibly collide with obstacles. (Default: None)
            executethread: (Default: None)
            executeITLOnCompleteLayerInfo: Execute ITL program after packing one layer of objects. (Default: None)
            executionVerificationInfo: Parameters for enabling and setting up execution verification that allows the robot to quickly stop when it is about to hit a part that is moved. This function is *highly recommend* to be used. (Default: None)
            needContainerResetMode: Enum that allows to reset needContainer signals in planning earlier while processing the last part. This will allow moveOut faster for that container if the condition is satisfied:
            - WaitForPickCompletion - waits for the pick to complete including all verification, this is the default setting.
            - ResetOnLastTargetInOrder - reset needContainer as soon as the last target in the order is picked up.
            - ResetOnNoMoreTargets - reset needContainer as soon as no more targets can be detected and last detected target is picked up.

            If checking or computing the container empty is set ("checkPickContainerEmptyOnFinish" or "computePickContainerEmptyOnFinish"), then will wait before can reset the pick container.
            If "canPlaceInSourceOnRecover"/"canPlaceInSourceOnBarcodeScanFail" is True, then will wait before can reset the pick container.
             (Default: None)
            feedbackDefaultJointThreshold: deg/mm, how many deg/mm each joint has to be from the commanded position to declare that it is reached. The smaller the value, the more accurate the robot will be, but will be slower. Used with feedbackDefaultWorkspaceThreshold. (Default: None)
            feedbackDefaultWorkspaceThreshold: mm, how many mm the real tooltip position has to be from the commanded position to declare that it is reached. The smaller the value, the more accurate the robot will be, but will be slower. Used with feedbackDestJointThreshold. (Default: None)
            feedbackDestJointThreshold: deg/mm, how many deg/mm each joint has to be from the commanded dest (release) position to declare that it is in at the destination. The smaller the value, the more accurate the robot will be, but will be slower. Used with feedbackDestWorkspaceThreshold. (Default: None)
            feedbackDestWorkspaceThreshold: mm, how many mm the real tooltip position has to be from the commanded dest (release) position to declare that it is in at the destination. The smaller the value, the more accurate the robot will be, but will be slower. Used with feedbackDestJointThreshold. (Default: None)
            feedbackGraspJointThreshold: deg/mm, how many deg/mm each joint has to be from the commanded grasp joint values to declare that it is at the grasp. The smaller the value, the more accurate the robot will be, but will be slower. Used with feedbackGraspWorkspaceThreshold. (Default: None)
            feedbackGraspWorkspaceThreshold: mm, how many mm the real tooltip position has to be from commanded grasp position to declare that it is at the grasp. The smaller the value, the more accurate the robot will be, but will be slower. Used with feedbackGraspJointThreshold. (Default: None)
            finalPlanMode: The mode to compute the final plan. If empty, do not do any final plan for each cycle, the robot stops after the dest depart. 

            If `cameraocclusion`, then make sure the robot ends the cycle in a position that does not cause camera occlusion.  
            If `config`, then will plan for robot 'finalPlan' only if the robot is occluding the camera at the source.  
            If `configIgnoreOcclusion`, then always plan to robot 'finalPlan' position.  
             (Default: None)
            finalPlanRobotConfiguration: The robot config for the final plan mode if finalPlanMode is config or configIgnoreOcclusion. (Default: None)
            filterGraspByUpPlaceOrientation: If True, only grasps facing upward will be considered during planning. Grasps where the approach direction is not aligned with the destination container's open face direction are filtered out.

            By default, this parameter is set to False, as many applications do not require perfectly aligned target placement.
            For example, if parts are dropped in the destination container, the grasp approach direction can be ignored and this parameter can be False.

            This parameter is used as an early termination flag, where the system can quickly discard invalid candidate grasp IKParams. This helps speed up the planning process by quickly checking the grasp's approach direction without needing to generate a grasp-goal pair and check validity for pairs that do not match the required placement positions. Other parameters can also affect early termination, such as `checkForEndEffectorLowerThanGraspDist`. (Default: None)
            finalPlanWaitTime: The time to wait after Final Plan, usually to allow system to rescan the environment.
             (Default: None)
            forceGraspModel: If True, forces the reuse of an already generated grasping model for picking the target object if the detected object size is within 1 cm in each axis from the registered part size.

            E.g. in palletizing applications, the system knows the incoming part sizes from the PLC for a sequence of items. Sometimes, the detected dimensions of the object are different from the specified part size, i.e. the object can be slightly smaller or larger than expected. This might require the system to recompute the grasping model, since it depends on the target object dimensions. (Default: None)
            forcetargetname: If not None, forces only one particular body to be picked up. forcetargetname has to match targetnamepattern. (Default: None)
            forceTargetNamePattern: If not None, forces particular bodies to be picked up. forceTargetNamePattern has to match to targetnamepattern. For picking from the buffer region. **Cannot** be used with forcetargetname. (Default: None)
            forceTargetUriCheck: If True, then will only pick up targets that match exactly with the specified target URI. (Default: None)
            forceSecondPreshapeAfterFirstApproach: If True, then after the robot finishes executing the first GraspApproach trajectory, the hand will move the gripper fingers to the second preshape configuration.

            This parameter can be useful for an application setup having a multi-joint gripper (e.g. shelf-container picking), that requires multiple grasp approach trajectories for picking the target object. In such cases, it is desirable to define a different preshape gripper configuration for each of the approach trajectories, thereby defining a multi-step gripper preshaping before execution. (Default: None)
            forceSecondReleaseAfterFirstDestDepart: If True, then after the robot finishes executing the first DestDepart trajectory, the hand will move the gripper fingers to the second release configuration.

            This parameter can be useful for an application setup having a multi-joint gripper (e.g. shelf-container picking), that requires multiple DestDepart trajectories after placing the target object. In such cases, it is needed to release the preshaped gripper configuration after each of the depart trajectories to make sure the gripper is ready for the next pick (Default: None)
            forceWaitDestContainer: If True, then wait for correct locationInfo from incoming destination containers, as it could have information about the partType, when no detection has started. This parameter will be applicable only when `useLocationState` is enabled.

            Example: If an AGV is managing the destination pallet in a palletizing application, the system needs to wait for the pallet to be placed in the correct location before starting the picking cycle.
             (Default: None)
            getCorrectPlanningReport: If True, computes a test pick and place command to give better error reporting to the user.

            The Binpicking planning loop may return `NoValidDest` during GrapGoalPair generation, even when there is no valid grasp. If this flag is enabled, `PlanToDestMode` is disabled, i.e. valid destinations are disabled, to confirm that a valid grasp can be generated. (Default: None)
            doSecondPreshapeAfterFirstApproach: If True, then after the robot finishes executing the first GraspApproach trajectory, the hand should move the gripper fingers to the second preshape configuration.

            This parameter can be useful for an application setup having a multi-joint gripper (e.g. shelf-container picking), that requires multiple grasp approach trajectories for picking the target object. In such cases, it may be desirable to define a different preshape gripper configuration for each of the approach trajectories, thereby defining a multi-step gripper preshaping before execution. (Default: None)
            doSecondReleaseAfterFirstDestDepart: If True, then after the robot finishes executing the first DestDepart trajectory, the hand should move the gripper fingers to the second release configuration.

            This parameter can be useful for an application setup having a multi-joint gripper (e.g. shelf-container picking), that requires multiple DestDepart trajectories after placing the target object. In such cases, it may be needed to release the preshaped gripper configuration after each of the depart trajectories to make sure the gripper is ready for the next pick (Default: None)
            releaseFingerOffsetAfterFirstDestDepart: Gripper finger values to adjust after finishing first DestDepart trajectory execution. This parameter defines how much farther away the gripper fingers should be moved, to achieve the required release configuration. (Default: None)
            grabbedTargetValidationSignalsInfo: (Default: None)
            graspApproachClearContainerTop: If True, will ensure that the first grasp approach starts from outside of the source container. This can make improve the likelihood of planning success under certain conditions, e.g. when a container is cluttered.

            Setting this parameter forces the robot to start approaching the Grasp from outside the container. This can increase the quality of the `Start` plan (shorter planning time, shorter execution time, smoother appearance, etc.)

            Example 1: Even if the source container is cluttered, the regular approach settings in `graspApproachInfos` may place the `graspApproachStart` point inside the container. As joint-space path planning is used for the `Start` motion which ends at the `graspApproachStart` point, the clutter can make motion planning difficult. In this case, turning this parameter on would make the `Start` motion easier to plan.

            Example 2: In the picture, the robot places a container in a shelf. It is better to enable this parameter and start to approach the Grasp from outside of the shelf container, as the clearance inside the shelf is too tight for joint-space path planning. (Default: None)
            graspDepartClearContainerTop: If True, will ensure that the last grasp depart finishes outside of the source container. This can make improve the likelihood of planning success under certain conditions, e.g. when a container is cluttered.

            Setting this parameter forces the robot to finish departing the Grasp from outside the container. This can increase the quality of the `transferToDest` plan (shorter planning time, shorter execution time, smoother appearance, etc.)

            Example 1: Even if the source container is cluttered, the regular depart settings in `graspDepartInfos` may place the `graspDepartEnd` point inside the container. As joint-space path planning is used for the `transferToDest` motion which starts at the `graspDepartEnd` point, the clutter can make motion planning difficult. In this case, turning this parameter on would make the `transferToDest` motion easier to plan.

            Example 2: In the picture, the robot places a container in a shelf. It is better to enable this parameter and finish departing from the Grasp to outside of the shelf container, as the clearance inside the shelf is too tight for joint-space path planning. (Default: None)
            graspDepartCurrentExceedThresholds: Absolute joint motor current thresholds that should not be exceeded. If exceeded, then will return torque error. (Default: None)
            graspDepartCurrentExceedThresholdsDelta: Check if difference between the start joint motor current values (at torqueCheckStartTime) and the current values exceed these thresholds. If they do, then will return torque error. (Default: None)
            graspApproachCollisionWallOffsetParameters: Parameters to move away from neighboring walls on departing. When these parameters are used to approaching, a movement is computed that starts at the given distance and moves closer to the walls. (Default: None)
            graspDepartAccelDecelScaleMultOnTargetMass: This array is used to compute a scaling multiplier that scales the grasp depart acceleration and deceleration from target mass. The computed scaling multiplier will be limited to a maximum of 1.0. This is useful when mass validation is enabled and computing accurate inertial force compensation is difficult. (Default: None)
            graspDepartReverseRecoveryDistance: mm, max distance to reverse the trajectory from the detected collision/error position when got torque error during grasp depart. -1 means that the default reverseRecoveryDistance from robot bridges should be used instead. (Default: None)
            graspDepartForceTorqueExceedThresholds: Absolute force and torque thresholds that should not be exceeded during grasp depart. Unit is N for force, Nm for torque. (Default: None)
            graspDepartForceTorqueExceedThresholdsDelta: Check if any of the force and torque exceed the difference between the values at start (at torqueCheckStartTime) and the current ones. If they do, then will return torque error. Unit is N for force, Nm for torque. (Default: None)
            graspDepartCollisionWallOffsetParameters: Parameters to move away from neighboring walls on departing. When these parameters are used to approaching, a movement is computed that starts at the given distance and moves closer to the walls. (Default: None)
            graspPenetrationOnTiltDist: mm. Distance to penetrate when robot is grasping a tilted part (Default: None)
            graspsetname: Name of the grasp set to use for picking. (Default: None)
            graspFilterByApproachOrientationThresh: degrees. This angle is used to filter grasps such that only approaches in the source container are allowed which are within a threshold of the container's up axis. If less than 0, then this setting is ignored. If absent, 90 is used instead. (Default: None)
            graspTimeLimit: Maximum time to spend on computing one grasp for one target. 0 means infinite. (Default: None)
            graspGoalPairCostMultipliers: Multipliers for various grasp-goal pair costs. (Default: None)
            graspPriorityMultipliers: Multipliers for various grasp priority values. (Default: None)
            heightDeltaUpdateOnNoMeasurement: m, Reduce the height of the target object by this value, since the robot did not go low enough to activate the line sensor. (Default: None)
            maxAcceptedDestPlanTrajTime: If > 0, then will only accept dest transfer plans (from source to dest container) that are faster than this limit. (Default: None)
            maxAcceptedFinishTrajTime: If > 0, then will only accept finish plans (for dest to finish) that are faster than this limit. (Default: None)
            heightIsAlwaysUncertain: (Default: None)
            maxCandidateMass: (Default: None)
            maxNumPackFormationSolutions: Maximum number of pack formation solutions to compute. (Default: None)
            moveToMidDestSkipWhenAllRegistered: If true, will skip moving to the mid destination if all parts are registered by auto-registration. (Default: None)
            ignoreDynamicObstaclesInGraspDepart: If true, will ignore any dynamic obstacles when departing after grasping. By default this is False since it is safer for the parts that the robot will be picking up, valid grasps will be harder to find though. (Default: None)
            ignoreFinishPosition: If True, will not move the robot to the finish position when the cycle finishes successfully. Note that when ignoreFinishPosition is True, the robot will still move to the finish position when production cycle is paused by users.. (Default: None)
            ignoreFinishPositionUnlessPackFormationComplete: If true, will ignore going to the finish position at the end of the cycle unless the robot is building a pack and it is complete at this cycle. (Default: None)
            ignoreIsPickable: If set to True, then can pick up any targets that match with targetnamepattern regardless of their `isPickable` flags. Set to False by default. (Default: None)
            ignoreMovableRobotNames: Set to other robots that are moving in the scene where planning should ignore their movable parts and interlock instead. (Default: None)
            iksolvername: IkSolver name. Choose between "ikfast" or "ikfastcpp". (Default: None)
            ikSolverParameters: (Default: None)
            iktimelimit: The timelimit in seconds to compute IK solutions until system starts the planning. If no IK solutions are computed by this time, then system will wait until at least one pair is computed, or everything is exhausted. (Default: None)
            ikTimeLimitForHighestPriority: If > 0, the timelimit in seconds to compute IK solutions for the highest priority targets. If IK solutions are computed for them, then start the grasp goal pairing right away to plan for it. (Default: None)
            inspectionFailDropOffInfo: Properties about drop off location (Default: None)
            initialDetectionValidationInfo: Specifies how to check condition of detected targets at the beginning of cycle. Number of detected targets and scanned barcode on detection time can be checked. (Default: None)
            initialMoveRobotOufOfCameraOcclusion: If True, then moves the robot out of camera occlusion before starting the cycle. (Default: None)
            inputPlacedPartInfoOnArrivals: Set of PlacedPartInfos that are on the destination container to be added into the environment (Default: None)
            ioSignalsInfo: (Default: None)
            isGripperSyncExecPossible: Indicates whether the gripper is capable of executing its motion synchronously with the arm motion. (Default: None)
            isTestMode: If true, runs system in test mode for testing pack formations. In test mode, packingGenerator will not be destroyed after computing packs. (Default: None)
            isStopOnGripperPositionNotReached: If True, stops the cycle execution if the gripper fails to move its fingers to a designated position. (Default: None)
            isStopOnObjectMassPropertiesMismatch: If True, stops the cycle execution if mass/center of mass validation reports mismatch. If False, cycle continues after mass mismatch only when robot can safely drop back an item to source. (Default: None)
            isStopOnPieceLost: If True and doAutoRecoveryOnPieceLost is False, stops the cycle execution if the piece is lost and raises an error. If doAutoRecoveryOnPieceLost is true and the piece was lost inside the source container (= the part most likely fell back inside), cycle execution continues. (Default: None)
            isStopOnRobotExecutionError: If True, stops the cycle execution if a robot execution error occurs and raise an error (Default: None)
            isStopOnTorqueLimitsError: If True, stops the cycle execution if the torque limits exceeded and raise an error (Default: None)
            isStopOnTorqueLimitsErrorOutsidePickLocation: If True, then stops the cycle execution if the torque limits exceeded outside of the source container. This usually when in mid-air, or at the destination, or somewhere during an intermediate cycle. (Default: None)
            isStopOnControllerError: If True, stops the cycle execution and raises an error if the robot has encountered a controller error. (Default: None)
            itlProgramNamesOnEvent: Names of ITL programs to run every cycle on specific events (Default: None)
            cyclePreconditionIOInfo: Info of precondition io checked before starting or resuming cycle (Default: None)
            logmessagesmask: If set to 1, log messages are stored for the user to see later. Should generally be disabled (0). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            justInTimeToolChangePlanning: Parameters for configuring just-in-time tool change planning feature (Default: None)
            labelPlacingInfo: Describes the label printer and how the robot computes the label placing motion. (Default: None)
            labelerDirection: The direction in which the labeling device (labeler) launches the labels. If the label is to be placed on the +X axis face of the target box, the labeler direction will point in the -X direction, e.g. a vector [-1,0,0]. (Default: None)
            localTargetDir: mm, When using 5D destination for target, this describes the axis in the local target coordinate system that should align with the global direction from the ikparam. (Default: None)
            logFailedTargetTimeout: seconds. If the workpiece failed to be picked up, then continue ignoring the workpiece for this many seconds. (Default: None)
            logFailedTargetPriorityTimeout: seconds. CURRENTLY NOT IMPLEMENTED. If the workpiece failed to be picked up, then this should decrease the priority of the workpiece for this duration. (Default: None)
            maxAllowedTargetSize: The maximum allowed size of the target to be picked up by planning. Should be defined in the object coordinate system. If the target is going to be out of this size in a certain target pose, it will be rejected.

            This parameter is mainly used for generating appropriate destination candidates and takes effect if `destcoordtype=targetAnyBottom...`. For example, in a depalletizing application that places boxes on a conveyor, the acceptable size may need to be limited to fit the conveyor. (Default: None)
            maxAllowedTargetSizeObjectName: Name of the object whose coordinate system will be used to compare the target size with `maxAllowedTargetSizeInDest`. If empty, then use world coordinate system.

            For example, in depalletizing applications, `maxAllowedTargetSize` may be based on conveyor coordinate system. (Default: None)
            maxConsideredCameraIkSolutions: Out of this many camera IK params that have IK solutions, will pick the one that gives solutions closest to the corresponding approach/depart configs. (Default: None)
            maxDestIkSolutions: The maximum IK solutions to have per destination before system gives up and tries something else. If 0, then compute all solutions (Default: None)
            maxGraspIkSolutions: The maximum IK solutions to have per target before system gives up and tries something else. If 0, then compute all solutions (Default: None)
            maxGraspsToConsider: If > 0, the max grasps of each grasp set to consider. If parameter is set too low, robot could finish with NoValidGrasp due to not considering the grasps. (Default: None)
            maxFinalPlanIgnoreCount: Max number of times final plan can be ignored. (Default: None)
            maxFinalPlanIgnoreMinTargets: minimum number of targets to have before can start to ignore the final plan mode (Default: None)
            maxGraspIkSolutionsPerGrasp: max grasp solutions allowed per grasp. If 0, then infinite (Default: None)
            maxIncidenceAngleOfIgnoreDestLinksForDestApproach: deg. If > 0, destination candidates with linear approach segments in which robot links with the _ignoredest_ tag collide obstacles and which incidence angle is larger than this number are rejected. This might be useful to prevent a suction foam from scratching work pieces already placed at a destination. (Default: None)
            maxIncidenceAngleOfIgnoreDestLinksForDestApproachIgnoreDistance: mm. If > 0, the constraint for maxIncidenceAngleOfIgnoreDestLinksForDestApproach is not applied when a target is within this range from its final position. (Default: None)
            maxStartFailuresForTargetGrasp: sometimes the start trajectory planning can fail because of a bad pair of grasps and targets. this controls how many failures are tolerated before system fails. (Default: None)
            maxStartFailuresForTarget: sometimes the start trajectory planning can fail because the target itself is in a corner or underneath other parts, and it shouldn't be picked up this cycle. This controls how many times a target failed to plan the start trajectory and then invalidates the target. (Default: None)
            maxLinearFailuresForTarget: sometimes the grasp approach/depart can fail if the target is still in collision with other objects, in that if the number of total failures for the target exceeds this threshold, should invalidate the target from consideration. (Default: None)
            maxLinearFailuresForTargetGrasp: sometimes the grasp approach/depart can fail if the target is still in collision with other objects, in that case all grasp departs will fail (Default: None)
            maxDestFailuresForTargetGrasp: sometimes the dest trajectory planning can fail because of a bad pair of grasps and targets. this controls how many failures are tolerated before system fails. (Default: None)
            maxTransferFailuresForTarget: sometimes transferring the target can fail because the target itself is in a corner or underneath other parts, and it shouldn't be picked up this cycle. This counts how many times a target failed to plan the tranfer trajectory and then invalidates the target. (Default: None)
            maxNumConsecutivePieceLost: If robot had PieceLost for amount of times in a row, then stop cycle (Default: None)
            maxNumConsecutiveCycleFailures: If set to a value > 0, then the cycle is stopped if the robot encountered this amount of cycle failures in a row. (Default: None)
            maxNumConsecutiveDistanceSensorFailures: If the system experienced distance sensor failure this amount of times consecutively (in a row), then the cycle is stopped. (Default: None)
            maxNumConsecutiveVerificationFailures: If set to a value > 0, then the cycle will stop if robot encounters this amount of execution verification failures in a row. (Default: None)
            maxNumConsecutiveRegistrationFailures: If set to a value > 0, then the cycle will stop if robot encounters this amount of registration failures in a row. (Default: None)
            maxNumConsecutiveSplitterOverweightFailures: If set to a value > 0, then the cycle will stop if robot encounters this amount of splitter sheet overweight failures in a row. (Default: None)
            maxNumPlanningFailedIterations: max number of planning iterations to fail until the system gives up (Default: None)
            maxPlanningCyclesToQueue: The max cycles to queue ahead for planning (Default: None)
            maxTimeForDecrease: The amount of countdown time (in seconds.) until the target, which previously failed, is clear from a penalty. This parameter is used in conjunction with targetPriorityMultipliers/failureCountdownMult. (Default: None)
            maxTorqueMultForApproach: the percentage of current to limit from maximum when grasp approaching. if 1, then disable (Default: None)
            maxTorqueMultForDestApproach: the percentage of current to limit from max when dest approaching. if 1, then disable (Default: None)
            mergeTrajectoryParametersForRecovery: Parameters for generating a connecting trajectory, which is a trajectory segment connecting the current robot values (which may be the result of the robot stopping due to io sensors) with the initial robot values of the next trajectory to be executed. (Default: None)
            midDestCoordType: coordinate system type of the mid-destination controlled by [midDestIkparamNames]. can be one of:
             - 'tool' specifying the current tool,
            - 'target' specifying the original target coordinate system. (Default: None)
            midDestIkparamNames: Names of the mid dest params to go to (Default: None)
            midDestPassthroughVelocity: The speed to pass through the mid-dest. (Default: None)
            midDestWaitTime: seconds, The time to wait at the middest (Default: None)
            maxManipAccel: If non-zero, then the Trajectory Timer considers the acceleration limit (m/s^2) of the active manipulators of the selected robots in the configuration space. Gravity is always included in the acceleration computations. 0 means no acceleration limit. (Default: None)
            maxManipSpeed: If non-zero, then the Trajectory Timer considers the speed limit (m/s) of the active manipulators of the selected robots in the configuration space. 0 means no speed limit. (Default: None)
            maxnotifygrasps: Specifies the number of grasp IK solutions to compute before notifying the other threads to start motion planning.

            If this parameter is set to 1, and the updated grasp IK solution is of low quality, planning might end up with a lot of grasp-goal pairs that have a low quality dest. The higher the number of 'maxnotifygrasps' and 'maxnotifydests', the better the probability of identifying good grasp-goal pairs, at the cost of longer planning time. (Default: None)
            maxnotifydests: Specifies the number of destination solutions to compute before the system starts creating planrequests by pairing grasp and destination solutions.

            If this parameter is set to 1, and the updated destination IK solution is of low quality, planning might end up with a lot of grasp-goal pairs that have a low quality dest. The higher the number of 'maxnotifygrasps' and 'maxnotifydests', the better the probability of identifying good grasp-goal pairs, at the cost of longer planning time. (Default: None)
            maxTargetNeighborsOnFace: If >= 0, will count how many neighbors target has and will reject the target as pickable if surpasses this number. For example to reject tagets that are completely surrounded, can set this value to 3. (Default: None)
            minGraspDepartCompleteRatio: Controls the percentage of the grasp depart trajectory that the robot needs to minimally achieve in order to move on. This can be used when grasp depart has to be big sometimes, but not always. By default it is 1. A lower value means that the robot will end the depart motion earlier and start the transfer motion. (Default: None)
            minNumSameConsecutiveDetections: When this parameter is set, only picks up detected targets if the detection is stable for this many number of detections. A value of 2 means target is pickable only if it is the same detection result between 2 images (Default: None)
            minViableRegionPlanParameters: parameters that are used with planning for minimum viable region (Default: None)
            moveStraightParams: Parameters used for linear movement like grasp approach, grasp depart, etc. (Default: None)
            moveStraightParamsOnReplan: Parameters used for linear movement when have to do dynamic replanning during execution. The parameters should be more loose than moveStraightParameters since the robot might be very close to thresholds and fail. (Default: None)
            moveToMidDest: If True, then will use 'midDestIkparamNames' to move to middest (Default: None)
            multiPickInfo: Parameters for Multi-Picking. Set mode=None to disable (Default: None)
            neighTargetThresh: A threshold to avoid picking multiple items close to one another.

            When using the same vision result to pick multiple items, objects which are closer to previous target objects than this threshold will not be considered. This means that all objects in a radius neighTargetThresh around picks will be discarded from the grasp candidates until a newer detection result is received. The threshold represents the area in which objects may have been moved by the picking actions, so that the vision result would not be reliable anymore.

            The distance is calculated between the object poses. It does not consider the object's mesh or shape.

            This parameter should be chosen depending on the type of object, the gripper and the vision system. For large object size and detection noise, a larger value will be appropriate. A reasonable value may be 0.5-2x the object's extent (the maximum dimension).
             (Default: None)
            neighTargetThreshForTorqueError: A threshold to avoid picking targets close to another target whose pick-place execution resulted in a torque limit error. 

            See the neighTargetThresh parameter for a detailed explanation. This parameter works the same way, but it only applies when the pick-place execution caused a torque limit error.

            A reasonable value may be 1-3x the object's extent.

            This parameter has no effect if it is smaller than neighTargetThresh.
             (Default: None)
            neighTargetThreshY: A threshold to avoid picking targets close to another target in the Y direction.

            See the neighTargetThresh parameter for a detailed explanation. This parameter works the same way, but the distance is calculated solely with regard to the position of the object in the Y-axis.

            This parameter should be chosen depending on the type of object, the gripper and the vision system. For large object sizes and significant detection noise, a larger value will be appropriate. A reasonable value may be 0.5-2x the object's Y-extent (the maximum dimension). (Default: None)
            notifyStateSlaveZMQUri: when a slave finishes the planning of the order cycle, will use this uri to notify another slave of the current state of the environment. Format is tcp://localhost:11000/?slaverequestid=XXX&username=YYY&otheroption&ZZZ (Default: None)
            numInitialDestinationResultsWorkers: Specifies the number of destination result worker threads that should be spawned. Initially, there should be as many worker threads as possible before hitting the IK limit. However, since the threads can be hard to cancel, the number should be limited. If 0, should be auto-set. (Default: None)
            numInitialGraspWorkers: Specifies the number of grasp worker threads that should be spawned. Initially, there should be as many worker threads as possible. However, since the threads can be hard to cancel, the number should be limited. If 0, should be auto-set. (Default: None)
            numInputPlacedPartInfoOnArrival: The number of parts to place in the destination. Items that were failed to be picked do not count. If 0, then will continue going until `cycledests` and `destikparamnames` have been all exhausted. Otherwise will only pick up these parts and stop the pick and place loop. (Default: None)
            numLogGraspPriorities: specifies the number of grasps shown in log to display priorities. (Default: None)
            numTargetsToPlace: The number of parts to place in the destination. picked up failures do not count. If 0, then will continue going until cycledests and destikparamnames have been all exhausted. Otherwise will only pick up these parts and stop the pick and place loop. (Default: None)
            numThreads: Number of threads to be used for planning (Default: None)
            numValidBodiesToTriggerWaitForCapturingBeforeGraspApproach: Forces the robot to wait outside the source container so that detection can be performed before performing another grasp.

            This parameter can be set if the picking motion is so fast that it interrupts image capture. In this case, it can increase performance if the robot waits outside of the source container for a short amount of time before picking the last valid candidate (by setting this parameter to 1). This allows detection to be performed while the robot is moving. Otherwise, the robot would have to wait for both the image capture and detection.

            If the number of valid bodies detected in the source container falls belows this number, the robot will move to a position outside of the occlusion region of the source container and wait for more detections by using the `WaitForCapturing` command. Disabled if set to -1. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            objectMassPropertiesCheckingInfo: Parameters to check object mass properties. If enabled, object mass properties will be checked dynamically while executing pick and place cycle. Force/Torque sensor is required to use this feature. (Default: None)
            objectTypeDynamicParametersMap: Describes some binpicking parameters which have to be applied for the specific objectType. (Default: None)
            targetOverlapConstraintInfo: Describes how overlapped regions in targets should be processed. (Default: None)
            packFormationComputationResult: A pack formation computed by Mujin. (Default: None)
            packFormationParameters: Parameters controlling the packing behaviors and algorithms for startPackFormationComputation command. (Default: None)
            orderPackFormationName: Specify a default pack formation name when running the Order Cycle. (Default: None)
            packLocationName: The default pack location name when running the Order Cycle. (Default: None)
            passOnDropAtDestinationNames: If not empty, then it is the name of a target object in the environment. If a part is dropped above this target object's aabb, then the part will be declared as success. By default it is empty. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            pickFailureDepartRetryNum: (Default: None)
            pickFailureDepartRetryWidth: (Default: None)
            pickLocationInfo: Describes the pick location and properties about it to initialize the cycle. (Default: None)
            pieceInspectionInfo: Piece inspection settings at the middest (Default: None)
            placedTargetPrefix: The prefix of all placed target names during the cycle (Default: None)
            placedTargetExtraPadding: mm, x,y,z. How much padding the robot needs to consider for lastly placed target. (Default: None)
            placedTargetRemainingTranslationRange: What translation (distance) range target remains after placing to the destination. mm, x,y,z. For example, this is useful when the destination is a conveyor and the conveyor will move placed target for certain range and need to protect target on that range. (Default: None)
            placeLocationInfos: Describes the place locations and properties about them to initialize the cycle. (Default: None)
            planningSchedulingMode: Controls the scheduling of internal grasp/dest planning into worker threads. (Default: None)
            planningSmallestObjectSizeForCollision: The smallest object size for collision detection while planning. (Default: None)
            planToDestMode: Controls how planning for destinations work. Can control which planning stages get planned for so that user can test various aspects of the plan.
            - None - then do not plan for any dest, and stop after the robot does a grasp depart
            - All - plan for everything
             (Default: None)
            predictDetectionInfo: Describes where to place predicted target before detection results. (Default: None)
            postCycleExecution: Post cycle execution and conveyor parameters. (Default: None)
            putBackParameters: Parameters to decide how slow put back motion is and how sensitive robot has to monitor FT sensor during put back.. (Default: None)
            randomBoxInfo: Info structure for maintaining grasp parameters for random box picking. Used when picking up randomized boxes (targetIsRandomBox is True). (Default: None)
            recoverySpeedMult: Speed multiplier when recovering from torque or piece-lost errors. (Default: None)
            rejectGraspOverlappingNonPickable: Mode of grasp rejecting for suction cups overlapping non-pickable objects. If enabled reject all the grasps that have suction cups that are overlapping with non-pickable targets, regardless of whether the suction cups are used or not.. (Default: None)
            restrictLabelOrientationInfo: Configuration of restricting label orientation of placed items. labelerDirection has to be defined for restriction to be applied. (Default: None)
            robotCycleStartPosition: mm or degrees. Need to specify values for all joints. (Default: None)
            robotFinishPosition: mm or degrees. Need to specify values for all joints. (Default: None)
            robotRecoveryPosition: mm or degrees. Need to specify values for all joints. (Default: None)
            robotRecoveryDepartAccel: mm/s^2  (Default: None)
            robotRecoveryDepartSpeed: mm/s (Default: None)
            robotRecoveryDepartOffsetDir: mm (x,y,z) (Default: None)
            robotRecoveryDepartOffsetInTool: If True, then robotRecoveryination depart offset is computed in the tool coordinate system (Default: None)
            saveConcatenateTrajectoryLog: Save trajectory logs which will be concatenated and scene file to reproduce the results. When this feature is enabled, the system can slow down a little since it is storing data. (Default: None)
            saveEnvState: If True, then saves the environment state for every plan. (Default: None)
            saveEnvStateOnUnexpectedFinish: Save environment state when the order cycle finishes unexpectedly. This flag is enabled by default in Production mode. (Default: None)
            saveGrabRecoveryScene: If true, then saves the environment whenever recovery occurs while robot is grabbing (GrabRecovery). (Default: None)
            savePlanningDiagnosticData: If true, will save internal PlanningDiagnosisData (PDD) files for every planning run for later root cause analysis. (Default: None)
            saveRecoveryScene: If true, then saves the environment whenever recovery from piece lost or other cancellation occurs. (Default: None)
            saveFilterTrajectoryLog: Save logs related to filtering in planning. When this feature is enabled, the system can slow down a little since it is storing data. (Default: None)
            saveOnPlanFailure: Save environment state when plan fails. Even if False, this flag will be enabled when the debug level is DEBUG or higher (Default: None)
            savetrajectorylog: True of False (Default: None)
            saveRobotFeedbackLog: Save logs from each trajectory the robot executes with data including the encoder values of the robot, the current/torque values, and specific IO signal values. When this feature is enabled, the system can slow down a little since it is storing data. Some UI functions that display data need this feature to be enabled so that the correct data can be displayed. (Default: None)
            saveVerificationScene: If True, then the environment state showing the robot execution verification failure is saved. (Default: None)
            saveWhenSlowPlanningDuration: Time duration in seconds, after which planning state will be saved, when planning is taking a long time. (Default: None)
            timeOutForFailedParts: ms. If the part failed within this time, do not consider it for the grasping stage. (Default: None)
            useKZFilter: Use KZ filter in FilterRobotTrajectory related functions. (Default: None)
            singularityEpsilon: radians. If the joint is within this threshold, it is assumed to be in a singularity. Joint positions near singularities are avoided during planning. (Default: None)
            skipCollidingDestsInfo: A dictionary specifying what kind of destinations can be skipped during destination validation process. (Default: None)
            skipLastImageCheckWhenNoMoreDestForDynamicGoals: If True, then after DynamicGoalGenerator returns with `NoMoreDest`, the last detection results from the source container are ignored - the last check is skipped.

            In a regular binpicking order cycle, even after the DynamicGoalGenerator has determined that there are no more valid destinations for target placement, the latest detection result could contain newly arrived targets in the source container which could be picked and placed in the destination container.

            By default, this parameter is set to False, so the order cycle will wait for the last detection results from the source container. (Default: None)
            skipTrajectoryPlanning: If True, the binpicking module skips trajectory planning and terminates when the first available plan request (grasp-goal pair) has been obtained. This is used to save calculation time for the layout simulator. (Default: None)
            splitterSheetInfo: A dictionary specifying parameters for splitter sheet picking. (Default: None)
            smootherParameters: Parameters dealing with getting smoother paths for the robot planning. (Default: None)
            sourceDynamicGoalsGeneratorParametersOverwrite: If 'useDynamicGoals' is True, then will be dynamically generating goals based how to call on the packing algorithms. Internally, the packing algorithms parameters are managed by packFormation profiles.
             (Default: None)
            sourcecameranames: list of cameras names for doing detection inside the source container. Each camera name is in the format of kinbody_name/attached_sensor_name. Overwritten by first layer value! (Default: None)
            sourcecontainername: (Default: None)
            sourceDestTargetOrientationPenalty: value multiplied by the rotational angle between the source and dest target orientations and added to the final cost of considering the particular grasp/goal pair. 0 if doesn't matter. By default it is 0 so not to penalize the orientations (Default: None)
            strictPickOrdering: Describes the order of targets to be picked up. Currently available strategies are:

            - 'axis'
            - 'containerLocalAxis'
            - 'name'
            - 'type'
             (Default: None)
            targetdestikthresh: threshold between expected target position from destinations with the planned target position after robot grabbed and transferred it. If the planned position of the target is within this threshold then assume that the plan is successful. Smaller the value more accurate the placement is, but fail more frequently. Default is 0.01 (Default: None)
            targetenvclearance: mm, environment clearacence applied to grabbed target (Default: None)
            targetGraspTimeLimit: sec, max time to spend on computing grasps for one target. 0 means infinite (Default: None)
            targetIsRandomBox: If True, then using randomized box picking which means vision will send results with the box sizes in it (Default: None)
            targetMinBottomPaddingForInitialTransfer: The amount of padding that is added to the bottom of the target when moving the target out of the source container to the next position (dest, middest, or scan position). This is used to raise the part higher when moving out of the source container.

            Increasing this parameter increases the clearance at the bottom of the part when it is moved out of the source container.

            Cannot be used together with targetMinSafetyHeightForInitialTransfer.

            Only applied during the initial transfer out of the source container. Subsequent transfers ignore this setting. (Default: None)
            targetMinSafetyHeightForInitialTransfer: Extends the height of the target to this value when moving the target out of the source container to the next position (dest, middest, or scan position). This is used to raise the part higher when moving out of the source container.

            Increasing this parameter increases the clearance at the bottom of the part when it is moved out of the source container.

            Cannot be used together with targetMinBottomPaddingForInitialTransfer.

            Only applied during the initial transfer out of the source container. Subsequent transfers ignore this setting. (Default: None)
            targetPaddingForGrasping: mm. Specifies the amount of padding to add to other dynamic targets (not related to the grasping target) when performing grasp generation.

            This ensures that the gripper maintains clearance to those targets, and avoids triggering execution verification with the point cloud unnecessarily. (Default: None)
            targetPlaceIkParamName: Name of the inverse kinematics parameter of the target to feet destination goal. Valid ONLY if destcoordtype=='targetplaceik'. If not defined, IK parameter name '_placed' will be used (Default: None)
            targetPlaceTranslationOffset: mm, x,y,z. A translation offset that is applied to the target in the simulated environment after placing. Most often represents the drop height at the destination. Should usually be set to the difference between the item in the gripper at the destination and after being released.

            Defined in the destination coordinate system. If the z-axis points upwards in the destination coordinate system, setting this parameter to [0, 0, -10] will move the simulated part to a position that is 10 mm lower.

            To adjust the height of the part at the destination, modify the destination goal instead of this parameter. (Default: None)
            targetPlaceTranslationOffsetInDest: If True, then the specified `targetPlaceTranslationOffset` parameter should be defined with respect to the destination container. This parameter is used to transform the `targetPlaceTranslationOffset` to the destination container orientation, if the destination container is defined in the environment.

            This parameter should be used with the `targetPlaceTranslationOffset` parameter. This parameter first checks if a destination container exists in the environment. If it does, the destination container orientation will be applied to the specified `targetPlaceTranslationOffset` vector. (Default: None)
            targetPostActionAfterPick: The action to perform on the target after picking it successfully from the pick (source) container.

            - **setUnpickable**: Do not delete target from the source container after it is picked up. The target is going to be removed only after update from vision. Not deleting a target is safer, but could slow the cycle time.
            - **moveDownSetUnpickable**: Move the part down and set as unpickable.
            - **delete**: Completely delete the part from the pick (source) container, as soon as it's planned for pickup. 
            - **deleteButConsiderForApproachMove**:
            - **deleteNotifyBridge**: Notify the robotbridge that the target was deleted from the location.
             (Default: None)
            targetPriorityMultipliers: Multipliers and other related quantities for various target priority values. (Default: None)
            targetStackDestHeightPenaltyMult: (Default: None)
            targetPrioritySuctionForceMult: Depending on how the gripper overlaps with the target object, it can exert different suction forces on it. This is a multiplier on the force (N) for a target's grasp priority. (Default: None)
            targetPriorityTransferSpeedMult: This is a multiplier on the transferSpeedMult (TSM) for a target's grasp priority. (Default: None)
            targeturi: Optional URI to use for every target. if randomized box picking, then leave empty (Default: None)
            targetThicknessThreshForTrapping: mm, threshold for maximum target thickness which can be trapped by disabled suction cup for items with objectType not 'bag' and 'twosidedpackage' items. Items with objectType 'bag' and 'twosidedpackage' will ignore such a parameter and will be considered for trapping. (Default: None)
            toolMaxRotationBetweenPickAndPlace: deg, If > 0, constraints the tool from rotating too much between pick and place positions. (Default: None)
            targetRotationConstraintParameters: Parameters for constraints to relative rotation between target at source and target at destination. Rotation here means rotation around world Z-axis. (Default: None)
            targetLabelAlignmentParameters: Parameters for constraints to align target label faces to a specific direction at a destination (Default: None)
            treatAsSquareTargetExtentsThreshold: mm, If > 0, when the difference of X and Y target edge lengths is smaller than this, the target is treated as square shaped and all placement directions are considered without trying to align a longer edge to X axis of a destination. (Default: None)
            toolPosConstraintPaddingXYZ: mm (x,y,z) (Default: None)
            toolposes: Tool poses for the robot. (Default: None)
            toolSpeedAccelInfo: Allows specifying limits on the speed and acceleration of the manipulator. (Default: None)
            toolSpeedAccelOptions: Used to apply the maxGrabbingManipAccel and maxFreeManipAccel during the pick and place cycle.

            - ** 0 **: Disabled.
            - ** 1 **: If set, then use fMaxManipSpeed/fMaxManipAccel for transferring when robot is grasping target.
            - ** 2 **: If set, then use fMaxManipSpeed/fMaxManipAccel for any other P2P movements.

            By default, this is disabled. (Default: None)
            torqueDistThresh: mm, when gravity compensation is enabled, how many mm offset the robot could get from its commanded position before declaring a torque limits error (Default: None)
            transferSpeedPostMult: Specifies the transfer speed mult ratio in [0,1] to reduce speed when grabbing an object. This is a global setting applied (Default: None)
            minAcceptedTransferSpeedMult: The minimum accepted combined transferSpeedMult. Reject everything else.

            Some grasps might have extremely slow speeds. This forces the robot to reject those grasps in order to save time. (Default: None)
            minTransferSpeedMult: Specifying the minimum transferSpeedMult ratio in [0,1]. If the resolved transferSpeedMult is less than this, but greater than minAcceptedTransferSpeedMult, then set the transferSpeedMult to this value. When there are multiple grasps with different transferSpeedMult (Default: None)
            transferSpeedMultPerWeight: An array of the transfer speed multipliers for different weights. If the partType's weight is between the weights in this array, the transfer speed multiplier (transferSpeedMult) is interpolated linearly. (Default: None)
            transferTrajectoryCostMultipliers: Multipliers for cost functions for transfer trajectories.

            Trajectory cost functions calculate a user-defined cost from the joint values at each discretized step along the trajectory. Each cost is multiplied with a multiplier defined in this object.

            This may be used to avoid e.g. joint configuration flips, which can cause "twisted"-looking and other disadvantageous trajectories. (Default: None)
            transferTrajectoryCostFnDiscretizationStep: seconds. Specifies the timestep to discretize an estimated transfer trajectory. The evaluation of user-defined cost value(s) is performed at each discretized point of the trajectory. 

            The finer the discretization step, the longer the computation time for the transfer trajectory plan. The larger the discretization step, the higher the possibility of computing a bad transfer trajectory plan. This parameter is mainly used for shelf-picking applications. (Default: None)
            updateHeightWithMeasuredValue: If True, update the height of the object depending on the distance sensor measurement. (Default: None)
            updateMassWithMeasuredValue: If True, update mass depending on the dynamic measurement of force torque sensor. (Default: None)
            unitMass: kg. Specifies the mass of the object used for packing (Default: None)
            useBarcodeUnpickableRegions: If True, then use barcode regions as unpickable regions for grasp sets. (Default: None)
            useDecreaseTargetEnvClearanceBigObjects: If True, then will decrease target environment clearance for objects which are bigger than source container inner region. (Default: None)
            useDetectedFace: If True, then will always use the detected face property of targets to prune grasps. If False, then it may be possible to pick up the target by using its non-detected faces.

            VisionTask usually returns the pose of the detected targets. `detectedFaces` is supplementary information about the target object, that the detector might provide. Planning uses this information to compute the best possible grasps of the target. 

            In picking scenarios, where the `detectedFace` is unpickable, and this parameter is False, the robot may pick up the target by using its non-detected faces. (Default: None)
            useDestGoalsInCollision: If True, then use destination goals even if they are in collision with the environment. Otherwise prune all dest goals that are in collision. During runtime, this ensures that the robot does not place a part in collision with other parts. (Default: None)
            useDropInSourceDests: If True, destinations that are for dropping in the source can be used. (Default: None)
            useDynamicGoals: If True, uses dynamic goals generation to define position of the items in the destcontainername. (Default: None)
            useExecutionQueueing: To use queueing to speed up robot execution, 0 to not use queueing (use for debugging since it is simpler) (Default: None)
            useFailThresholds: If True, the use the *OnFail looser overlap thresholds. Before going there, have to make sure that the vision data is latest and robot really doesn't have any other pick. Picking the last part can uncover a part underneath that is good. (Default: None)
            useLocationState: If true, then vision will track location IOs (containerType, containerId) from PLC for occlusion check or detection.

            By default, this is True. For testing, it can be set to False.

            This parameter is set to False at runtime when the system does not rely on location information for running picking cycles, for example in the following cases:
            - while running picking cycle in Editing mode
            - while running picking cycles without using production cycle
            - during preparation cycles
            - while computing pack formations (Default: None)
            waitDynamicGoalPointCloudTimeout: (Default: None)
            validatePlacedAllPickable: When set, will validate that all placed objects on the pack are still guaranteed that can be picked up by the gripper in any order. (Default: None)
            waitForBetterCostTime: seconds. The maximum additional time to wait after a valid plan result is found, but another plan request is still on-going and has a lower cost. If 0.0, the system will never wait for a better plan request to finish after a result has been found. (Default: None)
            waitForSupplyTimeout: sec. How long to wait before stopping the pick and place thread when container is not empty, but nothing is registered. This may need to be changed depending on the detection recognition time. (Default: None)
            waitForDetectionAfterInitialPick: If True, then will wait for new vision detection results for waitForSupplyTimeout after the initial pick. If False, will quit the cycle with FinishedNoMoreTargets after robot is done picking up everything that is detected, robot will not wait for vision results. This parameter is used in cases where speed is important and robot shouldn't waste time trying to pickup the last targets in the container. (Default: None)
            waitForDestProhibitedAtGrasp: If True, the robot will wait at the grasp position for the "dest prohibited" PLC signal to go to OFF before starting the part transfer motion. If False, the robot leaves immediately after grasping but may stop mid-way if the destination is not ready (interlock).

            Setting this to True is safer if stopping mid-way may cause the part to drop or if the robot does not have enough time to stop before entering the interlock region. (Default: None)
            waitForSourceProhibitedAtDest: If True, robot will wait for source prohibited PLC signal to go to OFF after placing a part at the dest before going home or doing other recovery tasks. This feature ensures that the robot doesn't do any interlocking in the middle that could have the robot decelerating fast and stopping (especially for big robots). By default this is False. (Default: None)
            waitForStateTrigger: Wait for this finish code from another slave's to trigger this slave's computation (Default: None)
            waitForLocationUnprohibited: Wait for all location to become unprohibited. (Default: None)
            waitUpdateStampTimeout: sec, how long to wait for the initial vision results to come before starting pick and place (Default: None)
            ykkControlInfo: Information on controlling ykk external devices (Default: None)
            containername: (Default: None)
            controllerclientparameters: (Default: None)
            cycleIndex: (Default: None)
            cycleStartUseToolPose: True if the robot should go to the tool position rather than joint values at the start of the cycle (Default: False)
            destGoals: list of dictionaries where each dictionary contains the goal description. A goal contains: ikparamnames or jointvalues, validGraspSetName(optional), name(optional). (Default: None)
            destcontainernames: (Default: None)
            destSensorSelectionInfos: (Default: None)
            detectionInfos: (Default: None)
            forceMoveToFinish: If True, then the robot will add a finish position to the cycle even if "finish" is not present, unless "ignoreFinishPosition" is True in the order cycle command. "ignoreFinishPosition" overrides this parameter. (Default: None)
            forceStartRobotPositionConfigurationName: If not None, then have the robot start with this position configuration regardless of what is in orderIds or robot positions/connected body active states. (Default: None)
            ignoreStartPosition: True if the robot should ignore going to start position (Default: None)
            initiallyDisableRobotBridge: If True, stops any communication with the robotbridge until robot bridge is enabled. (Default: None)
            inputPartIndex: (Default: None)
            itlParameters: (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            pickContainerHasOnlyOnePart: If True, assumes there is only one part in source container regardless of orderNumInPickContainer in order request (Default: False)
            placedTargetPrefixes: (Default: None)
            registrationInfo: (Default: None)
            sourcecontainernames: (Default: None)
            sourceSensorSelectionInfos: (Default: None)
            targetname: (Default: None)
        """
        assert (targetnamepattern is not None)
        if regionname is None:
            regionname = self.regionname
        taskparameters = {
            'command': 'StartPickAndPlaceThread',
            'targetnamepattern': targetnamepattern,
            'approachoffset': approachoffset,
            'departoffsetdir': departoffsetdir,
            'destdepartoffsetdir': destdepartoffsetdir,
            'deletetarget': deletetarget,
            'debuglevel': debuglevel,
            'movetodestination': movetodestination,
            'regionname': regionname,
            'envclearance': envclearance,
            'containername': containername,
        }  # type: dict[str, Any]
        if goals is not None:
            taskparameters['goaltype'] = goaltype
        if goals is not None:
            taskparameters['orderedgoals'] = goals
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if allTargetsDifferentUri is not None:
            taskparameters['allTargetsDifferentUri'] = allTargetsDifferentUri
        if absMaxPlanningTimeToWait is not None:
            taskparameters['absMaxPlanningTimeToWait'] = absMaxPlanningTimeToWait
        if addGraspGoalPairWorker is not None:
            taskparameters['addGraspGoalPairWorker'] = addGraspGoalPairWorker
        if alwaysPlanOutOfOcclusion is not None:
            taskparameters['alwaysPlanOutOfOcclusion'] = alwaysPlanOutOfOcclusion
        if approachCurrentExceedThresholds is not None:
            taskparameters['approachCurrentExceedThresholds'] = approachCurrentExceedThresholds
        if approachCurrentExceedThresholdsDelta is not None:
            taskparameters['approachCurrentExceedThresholdsDelta'] = approachCurrentExceedThresholdsDelta
        if approachForceTorqueExceedThresholds is not None:
            taskparameters['approachForceTorqueExceedThresholds'] = approachForceTorqueExceedThresholds
        if approachForceTorqueExceedThresholdsDelta is not None:
            taskparameters['approachForceTorqueExceedThresholdsDelta'] = approachForceTorqueExceedThresholdsDelta
        if approachOffsetFromWalls is not None:
            taskparameters['approachOffsetFromWalls'] = approachOffsetFromWalls
        if atStartPlanDynamicContentsNames is not None:
            taskparameters['atStartPlanDynamicContentsNames'] = atStartPlanDynamicContentsNames
        if automaticToolPosConstraintNotGrabbing is not None:
            taskparameters['automaticToolPosConstraintNotGrabbing'] = automaticToolPosConstraintNotGrabbing
        if automaticToolPosConstraintWhenGrabbing is not None:
            taskparameters['automaticToolPosConstraintWhenGrabbing'] = automaticToolPosConstraintWhenGrabbing
        if barcodeScanningInfo is not None:
            taskparameters['barcodeScanningInfo'] = barcodeScanningInfo
        if binpickingDebugMode is not None:
            taskparameters['binpickingDebugMode'] = binpickingDebugMode
        if bodyNameToAvoidFinalCollision is not None:
            taskparameters['bodyNameToAvoidFinalCollision'] = bodyNameToAvoidFinalCollision
        if bottomScanPlacementInfo is not None:
            taskparameters['bottomScanPlacementInfo'] = bottomScanPlacementInfo
        if cameraPlanningInfos is not None:
            taskparameters['cameraPlanningInfos'] = cameraPlanningInfos
        if cameraOcclusionApplyGrabbedState is not None:
            taskparameters['cameraOcclusionApplyGrabbedState'] = cameraOcclusionApplyGrabbedState
        if cameraOcclusionOffset is not None:
            taskparameters['cameraOcclusionOffset'] = cameraOcclusionOffset
        if cameraOcclusionPaddingTime is not None:
            taskparameters['cameraOcclusionPaddingTime'] = cameraOcclusionPaddingTime
        if cameraOcclusionPaddingTimeStart is not None:
            taskparameters['cameraOcclusionPaddingTimeStart'] = cameraOcclusionPaddingTimeStart
        if cameraOcclusionPaddingTimeEnd is not None:
            taskparameters['cameraOcclusionPaddingTimeEnd'] = cameraOcclusionPaddingTimeEnd
        if cameraOcclusionUseLinkVisibility is not None:
            taskparameters['cameraOcclusionUseLinkVisibility'] = cameraOcclusionUseLinkVisibility
        if canPlaceInSourceOnRecover is not None:
            taskparameters['canPlaceInSourceOnRecover'] = canPlaceInSourceOnRecover
        if canPlaceInSourceOnBarcodeScanFail is not None:
            taskparameters['canPlaceInSourceOnBarcodeScanFail'] = canPlaceInSourceOnBarcodeScanFail
        if canSetIsPickable is not None:
            taskparameters['canSetIsPickable'] = canSetIsPickable
        if checkCameraOcclusionAtMidDest is not None:
            taskparameters['checkCameraOcclusionAtMidDest'] = checkCameraOcclusionAtMidDest
        if checkCollisionAtDestNames is not None:
            taskparameters['checkCollisionAtDestNames'] = checkCollisionAtDestNames
        if checkDestContainerEmptyOnArrivalNames is not None:
            taskparameters['checkDestContainerEmptyOnArrivalNames'] = checkDestContainerEmptyOnArrivalNames
        if checkExpectedDetectedHeightThreshold is not None:
            taskparameters['checkExpectedDetectedHeightThreshold'] = checkExpectedDetectedHeightThreshold
        if checkForEndEffectorLowerThanGraspDist is not None:
            taskparameters['checkForEndEffectorLowerThanGraspDist'] = checkForEndEffectorLowerThanGraspDist
        if checkObstacleNames is not None:
            taskparameters['checkObstacleNames'] = checkObstacleNames
        if checkPickContainerEmptyOnFinish is not None:
            taskparameters['checkPickContainerEmptyOnFinish'] = checkPickContainerEmptyOnFinish
        if checkPlaceContainerEmptyOnArrival is not None:
            taskparameters['checkPlaceContainerEmptyOnArrival'] = checkPlaceContainerEmptyOnArrival
        if clearanceTopDestContainer is not None:
            taskparameters['clearanceTopDestContainer'] = clearanceTopDestContainer
        if clearanceTopSourceContainer is not None:
            taskparameters['clearanceTopSourceContainer'] = clearanceTopSourceContainer
        if computePickContainerEmptyOnFinish is not None:
            taskparameters['computePickContainerEmptyOnFinish'] = computePickContainerEmptyOnFinish
        if computePickContainerDamagedOnFinish is not None:
            taskparameters['computePickContainerDamagedOnFinish'] = computePickContainerDamagedOnFinish
        if constraintToolInfo is not None:
            taskparameters['constraintToolInfo'] = constraintToolInfo
        if constraintToolPosToSource is not None:
            taskparameters['constraintToolPosToSource'] = constraintToolPosToSource
        if containerEmptyWaitTime is not None:
            taskparameters['containerEmptyWaitTime'] = containerEmptyWaitTime
        if cycledests is not None:
            taskparameters['cycledests'] = cycledests
        if deleteDynamicObjectsAfterFinish is not None:
            taskparameters['deleteDynamicObjectsAfterFinish'] = deleteDynamicObjectsAfterFinish
        if deleteTargetWhenPlacedInDest is not None:
            taskparameters['deleteTargetWhenPlacedInDest'] = deleteTargetWhenPlacedInDest
        if deleteTargetFromSourceContainer is not None:
            taskparameters['deleteTargetFromSourceContainer'] = deleteTargetFromSourceContainer
        if destikparamnames is not None:
            taskparameters['destikparamnames'] = destikparamnames
        if intermediateCycles is not None:
            taskparameters['intermediateCycles'] = intermediateCycles
        if deleteTargetsOnPieceLost is not None:
            taskparameters['deleteTargetsOnPieceLost'] = deleteTargetsOnPieceLost
        if deleteTargetsOnRecovery is not None:
            taskparameters['deleteTargetsOnRecovery'] = deleteTargetsOnRecovery
        if deleteTargetsEveryCycleSlowMode is not None:
            taskparameters['deleteTargetsEveryCycleSlowMode'] = deleteTargetsEveryCycleSlowMode
        if destBarcodeScanningInfo is not None:
            taskparameters['destBarcodeScanningInfo'] = destBarcodeScanningInfo
        if destBarcodeScanningInfoPerContainer is not None:
            taskparameters['destBarcodeScanningInfoPerContainer'] = destBarcodeScanningInfoPerContainer
        if destTargetAABBAlignIkParameters is not None:
            taskparameters['destTargetAABBAlignIkParameters'] = destTargetAABBAlignIkParameters
        if destTargetAnyBottomFaceRotationParameters is not None:
            taskparameters['destTargetAnyBottomFaceRotationParameters'] = destTargetAnyBottomFaceRotationParameters
        if destTargetCornerParameters is not None:
            taskparameters['destTargetCornerParameters'] = destTargetCornerParameters
        if destTargetStackParameters is not None:
            taskparameters['destTargetStackParameters'] = destTargetStackParameters
        if dynamicGoalsParameters is not None:
            taskparameters['dynamicGoalsParameters'] = dynamicGoalsParameters
        if ensurePickVisibilityAtPlace is not None:
            taskparameters['ensurePickVisibilityAtPlace'] = ensurePickVisibilityAtPlace
        if graspApproachInfos is not None:
            taskparameters['graspApproachInfos'] = graspApproachInfos
        if graspApproachInfosPerURI is not None:
            taskparameters['graspApproachInfosPerURI'] = graspApproachInfosPerURI
        if graspDepartInfos is not None:
            taskparameters['graspDepartInfos'] = graspDepartInfos
        if graspDepartAboveNearbyObstacles is not None:
            taskparameters['graspDepartAboveNearbyObstacles'] = graspDepartAboveNearbyObstacles
        if graspDepartAboveNearbyObstaclesMaxDist is not None:
            taskparameters['graspDepartAboveNearbyObstaclesMaxDist'] = graspDepartAboveNearbyObstaclesMaxDist
        if departOffsetFromWalls is not None:
            taskparameters['departOffsetFromWalls'] = departOffsetFromWalls
        if destApproachCurrentExceedThresholds is not None:
            taskparameters['destApproachCurrentExceedThresholds'] = destApproachCurrentExceedThresholds
        if destApproachCurrentExceedThresholdsDelta is not None:
            taskparameters['destApproachCurrentExceedThresholdsDelta'] = destApproachCurrentExceedThresholdsDelta
        if destApproachForceTorqueExceedThresholds is not None:
            taskparameters['destApproachForceTorqueExceedThresholds'] = destApproachForceTorqueExceedThresholds
        if destApproachForceTorqueExceedThresholdsDelta is not None:
            taskparameters['destApproachForceTorqueExceedThresholdsDelta'] = destApproachForceTorqueExceedThresholdsDelta
        if destApproachClearContainerTop is not None:
            taskparameters['destApproachClearContainerTop'] = destApproachClearContainerTop
        if destApproachInfos is not None:
            taskparameters['destApproachInfos'] = destApproachInfos
        if destApproachAccelDecelScaleMultOnTargetMass is not None:
            taskparameters['destApproachAccelDecelScaleMultOnTargetMass'] = destApproachAccelDecelScaleMultOnTargetMass
        if destApproachAccelDecelMultOnTilt is not None:
            taskparameters['destApproachAccelDecelMultOnTilt'] = destApproachAccelDecelMultOnTilt
        if destApproachSpeedMultOnTilt is not None:
            taskparameters['destApproachSpeedMultOnTilt'] = destApproachSpeedMultOnTilt
        if destApproachSpeedMultOnUnknownSize is not None:
            taskparameters['destApproachSpeedMultOnUnknownSize'] = destApproachSpeedMultOnUnknownSize
        if destcontainername is not None:
            taskparameters['destcontainername'] = destcontainername
        if destcoordtype is not None:
            taskparameters['destcoordtype'] = destcoordtype
        if destDepartFutureCycleMerge is not None:
            taskparameters['destDepartFutureCycleMerge'] = destDepartFutureCycleMerge
        if destDepartInfos is not None:
            taskparameters['destDepartInfos'] = destDepartInfos
        if destDepartSpeedMultOnUnknownSize is not None:
            taskparameters['destDepartSpeedMultOnUnknownSize'] = destDepartSpeedMultOnUnknownSize
        if destDepartClearContainerTop is not None:
            taskparameters['destDepartClearContainerTop'] = destDepartClearContainerTop
        if destFilterByTargetOrientationThresh is not None:
            taskparameters['destFilterByTargetOrientationThresh'] = destFilterByTargetOrientationThresh
        if destFilterByTargetOrientationThreshPerMass is not None:
            taskparameters['destFilterByTargetOrientationThreshPerMass'] = destFilterByTargetOrientationThreshPerMass
        if destPriorityLabelDirectionValue is not None:
            taskparameters['destPriorityLabelDirectionValue'] = destPriorityLabelDirectionValue
        if destTargetValidationJitterDist is not None:
            taskparameters['destTargetValidationJitterDist'] = destTargetValidationJitterDist
        if detectionResultsMaxCacheTime is not None:
            taskparameters['detectionResultsMaxCacheTime'] = detectionResultsMaxCacheTime
        if discardTargetIfJitterFails is not None:
            taskparameters['discardTargetIfJitterFails'] = discardTargetIfJitterFails
        if discardTargetIfMergeAfterJitterFails is not None:
            taskparameters['discardTargetIfMergeAfterJitterFails'] = discardTargetIfMergeAfterJitterFails
        if disablePlacedTargetsInPickLocationWhenPlanning is not None:
            taskparameters['disablePlacedTargetsInPickLocationWhenPlanning'] = disablePlacedTargetsInPickLocationWhenPlanning
        if disableTargetCheckOnGraspApproach is not None:
            taskparameters['disableTargetCheckOnGraspApproach'] = disableTargetCheckOnGraspApproach
        if disableUnchuck is not None:
            taskparameters['disableUnchuck'] = disableUnchuck
        if detectionTriggerMode is not None:
            taskparameters['detectionTriggerMode'] = detectionTriggerMode
        if distanceMeasurementInfo is not None:
            taskparameters['distanceMeasurementInfo'] = distanceMeasurementInfo
        if doAccurateGraspDepart is not None:
            taskparameters['doAccurateGraspDepart'] = doAccurateGraspDepart
        if doAutoRecoveryOnPieceLost is not None:
            taskparameters['doAutoRecoveryOnPieceLost'] = doAutoRecoveryOnPieceLost
        if doAutoRecoveryOnRobotExecutionError is not None:
            taskparameters['doAutoRecoveryOnRobotExecutionError'] = doAutoRecoveryOnRobotExecutionError
        if doSimultaneousGripperPreshapingDuringApproach is not None:
            taskparameters['doSimultaneousGripperPreshapingDuringApproach'] = doSimultaneousGripperPreshapingDuringApproach
        if dropInDestInfo is not None:
            taskparameters['dropInDestInfo'] = dropInDestInfo
        if dropInDestInfoPerContainer is not None:
            taskparameters['dropInDestInfoPerContainer'] = dropInDestInfoPerContainer
        if dropOffParameters is not None:
            taskparameters['dropOffParameters'] = dropOffParameters
        if dropOffsetFromCollision is not None:
            taskparameters['dropOffsetFromCollision'] = dropOffsetFromCollision
        if dropTargetInSourceContainerBoxMult is not None:
            taskparameters['dropTargetInSourceContainerBoxMult'] = dropTargetInSourceContainerBoxMult
        if dropTargetInDestContainerBoxMult is not None:
            taskparameters['dropTargetInDestContainerBoxMult'] = dropTargetInDestContainerBoxMult
        if dropTargetInDestContainerZSafetyMult is not None:
            taskparameters['dropTargetInDestContainerZSafetyMult'] = dropTargetInDestContainerZSafetyMult
        if dropTargetMaxDistanceThresold is not None:
            taskparameters['dropTargetMaxDistanceThresold'] = dropTargetMaxDistanceThresold
        if dropTargetMaxDistanceXYThreshold is not None:
            taskparameters['dropTargetMaxDistanceXYThreshold'] = dropTargetMaxDistanceXYThreshold
        if enableBodyNamesOnCameraPlan is not None:
            taskparameters['enableBodyNamesOnCameraPlan'] = enableBodyNamesOnCameraPlan
        if enableBodyNamesOnDestPlan is not None:
            taskparameters['enableBodyNamesOnDestPlan'] = enableBodyNamesOnDestPlan
        if enableBodyNamesOnStartPlan is not None:
            taskparameters['enableBodyNamesOnStartPlan'] = enableBodyNamesOnStartPlan
        if encoderConvergenceSpeedThresh is not None:
            taskparameters['encoderConvergenceSpeedThresh'] = encoderConvergenceSpeedThresh
        if executionConnectingTrajDecelMult is not None:
            taskparameters['executionConnectingTrajDecelMult'] = executionConnectingTrajDecelMult
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if executionEnvClearanceApproachConcatenate is not None:
            taskparameters['executionEnvClearanceApproachConcatenate'] = executionEnvClearanceApproachConcatenate
        if executionMaxConcatenateSearchTime is not None:
            taskparameters['executionMaxConcatenateSearchTime'] = executionMaxConcatenateSearchTime
        if executionConcatenateSearchDeltaTime is not None:
            taskparameters['executionConcatenateSearchDeltaTime'] = executionConcatenateSearchDeltaTime
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if executionFilterFactorWhenGrabbing is not None:
            taskparameters['executionFilterFactorWhenGrabbing'] = executionFilterFactorWhenGrabbing
        if executethread is not None:
            taskparameters['executethread'] = executethread
        if executeITLOnCompleteLayerInfo is not None:
            taskparameters['executeITLOnCompleteLayerInfo'] = executeITLOnCompleteLayerInfo
        if executionVerificationInfo is not None:
            taskparameters['executionVerificationInfo'] = executionVerificationInfo
        if needContainerResetMode is not None:
            taskparameters['needContainerResetMode'] = needContainerResetMode
        if feedbackDefaultJointThreshold is not None:
            taskparameters['feedbackDefaultJointThreshold'] = feedbackDefaultJointThreshold
        if feedbackDefaultWorkspaceThreshold is not None:
            taskparameters['feedbackDefaultWorkspaceThreshold'] = feedbackDefaultWorkspaceThreshold
        if feedbackDestJointThreshold is not None:
            taskparameters['feedbackDestJointThreshold'] = feedbackDestJointThreshold
        if feedbackDestWorkspaceThreshold is not None:
            taskparameters['feedbackDestWorkspaceThreshold'] = feedbackDestWorkspaceThreshold
        if feedbackGraspJointThreshold is not None:
            taskparameters['feedbackGraspJointThreshold'] = feedbackGraspJointThreshold
        if feedbackGraspWorkspaceThreshold is not None:
            taskparameters['feedbackGraspWorkspaceThreshold'] = feedbackGraspWorkspaceThreshold
        if finalPlanMode is not None:
            taskparameters['finalPlanMode'] = finalPlanMode
        if finalPlanRobotConfiguration is not None:
            taskparameters['finalPlanRobotConfiguration'] = finalPlanRobotConfiguration
        if filterGraspByUpPlaceOrientation is not None:
            taskparameters['filterGraspByUpPlaceOrientation'] = filterGraspByUpPlaceOrientation
        if finalPlanWaitTime is not None:
            taskparameters['finalPlanWaitTime'] = finalPlanWaitTime
        if forceGraspModel is not None:
            taskparameters['forceGraspModel'] = forceGraspModel
        if forcetargetname is not None:
            taskparameters['forcetargetname'] = forcetargetname
        if forceTargetNamePattern is not None:
            taskparameters['forceTargetNamePattern'] = forceTargetNamePattern
        if forceTargetUriCheck is not None:
            taskparameters['forceTargetUriCheck'] = forceTargetUriCheck
        if forceSecondPreshapeAfterFirstApproach is not None:
            taskparameters['forceSecondPreshapeAfterFirstApproach'] = forceSecondPreshapeAfterFirstApproach
        if forceSecondReleaseAfterFirstDestDepart is not None:
            taskparameters['forceSecondReleaseAfterFirstDestDepart'] = forceSecondReleaseAfterFirstDestDepart
        if forceWaitDestContainer is not None:
            taskparameters['forceWaitDestContainer'] = forceWaitDestContainer
        if getCorrectPlanningReport is not None:
            taskparameters['getCorrectPlanningReport'] = getCorrectPlanningReport
        if doSecondPreshapeAfterFirstApproach is not None:
            taskparameters['doSecondPreshapeAfterFirstApproach'] = doSecondPreshapeAfterFirstApproach
        if doSecondReleaseAfterFirstDestDepart is not None:
            taskparameters['doSecondReleaseAfterFirstDestDepart'] = doSecondReleaseAfterFirstDestDepart
        if releaseFingerOffsetAfterFirstDestDepart is not None:
            taskparameters['releaseFingerOffsetAfterFirstDestDepart'] = releaseFingerOffsetAfterFirstDestDepart
        if grabbedTargetValidationSignalsInfo is not None:
            taskparameters['grabbedTargetValidationSignalsInfo'] = grabbedTargetValidationSignalsInfo
        if graspApproachClearContainerTop is not None:
            taskparameters['graspApproachClearContainerTop'] = graspApproachClearContainerTop
        if graspDepartClearContainerTop is not None:
            taskparameters['graspDepartClearContainerTop'] = graspDepartClearContainerTop
        if graspDepartCurrentExceedThresholds is not None:
            taskparameters['graspDepartCurrentExceedThresholds'] = graspDepartCurrentExceedThresholds
        if graspDepartCurrentExceedThresholdsDelta is not None:
            taskparameters['graspDepartCurrentExceedThresholdsDelta'] = graspDepartCurrentExceedThresholdsDelta
        if graspApproachCollisionWallOffsetParameters is not None:
            taskparameters['graspApproachCollisionWallOffsetParameters'] = graspApproachCollisionWallOffsetParameters
        if graspDepartAccelDecelScaleMultOnTargetMass is not None:
            taskparameters['graspDepartAccelDecelScaleMultOnTargetMass'] = graspDepartAccelDecelScaleMultOnTargetMass
        if graspDepartReverseRecoveryDistance is not None:
            taskparameters['graspDepartReverseRecoveryDistance'] = graspDepartReverseRecoveryDistance
        if graspDepartForceTorqueExceedThresholds is not None:
            taskparameters['graspDepartForceTorqueExceedThresholds'] = graspDepartForceTorqueExceedThresholds
        if graspDepartForceTorqueExceedThresholdsDelta is not None:
            taskparameters['graspDepartForceTorqueExceedThresholdsDelta'] = graspDepartForceTorqueExceedThresholdsDelta
        if graspDepartCollisionWallOffsetParameters is not None:
            taskparameters['graspDepartCollisionWallOffsetParameters'] = graspDepartCollisionWallOffsetParameters
        if graspPenetrationOnTiltDist is not None:
            taskparameters['graspPenetrationOnTiltDist'] = graspPenetrationOnTiltDist
        if graspsetname is not None:
            taskparameters['graspsetname'] = graspsetname
        if graspFilterByApproachOrientationThresh is not None:
            taskparameters['graspFilterByApproachOrientationThresh'] = graspFilterByApproachOrientationThresh
        if graspTimeLimit is not None:
            taskparameters['graspTimeLimit'] = graspTimeLimit
        if graspGoalPairCostMultipliers is not None:
            taskparameters['graspGoalPairCostMultipliers'] = graspGoalPairCostMultipliers
        if graspPriorityMultipliers is not None:
            taskparameters['graspPriorityMultipliers'] = graspPriorityMultipliers
        if heightDeltaUpdateOnNoMeasurement is not None:
            taskparameters['heightDeltaUpdateOnNoMeasurement'] = heightDeltaUpdateOnNoMeasurement
        if maxAcceptedDestPlanTrajTime is not None:
            taskparameters['maxAcceptedDestPlanTrajTime'] = maxAcceptedDestPlanTrajTime
        if maxAcceptedFinishTrajTime is not None:
            taskparameters['maxAcceptedFinishTrajTime'] = maxAcceptedFinishTrajTime
        if heightIsAlwaysUncertain is not None:
            taskparameters['heightIsAlwaysUncertain'] = heightIsAlwaysUncertain
        if maxCandidateMass is not None:
            taskparameters['maxCandidateMass'] = maxCandidateMass
        if maxNumPackFormationSolutions is not None:
            taskparameters['maxNumPackFormationSolutions'] = maxNumPackFormationSolutions
        if moveToMidDestSkipWhenAllRegistered is not None:
            taskparameters['moveToMidDestSkipWhenAllRegistered'] = moveToMidDestSkipWhenAllRegistered
        if ignoreDynamicObstaclesInGraspDepart is not None:
            taskparameters['ignoreDynamicObstaclesInGraspDepart'] = ignoreDynamicObstaclesInGraspDepart
        if ignoreFinishPosition is not None:
            taskparameters['ignoreFinishPosition'] = ignoreFinishPosition
        if ignoreFinishPositionUnlessPackFormationComplete is not None:
            taskparameters['ignoreFinishPositionUnlessPackFormationComplete'] = ignoreFinishPositionUnlessPackFormationComplete
        if ignoreIsPickable is not None:
            taskparameters['ignoreIsPickable'] = ignoreIsPickable
        if ignoreMovableRobotNames is not None:
            taskparameters['ignoreMovableRobotNames'] = ignoreMovableRobotNames
        if iksolvername is not None:
            taskparameters['iksolvername'] = iksolvername
        if ikSolverParameters is not None:
            taskparameters['ikSolverParameters'] = ikSolverParameters
        if iktimelimit is not None:
            taskparameters['iktimelimit'] = iktimelimit
        if ikTimeLimitForHighestPriority is not None:
            taskparameters['ikTimeLimitForHighestPriority'] = ikTimeLimitForHighestPriority
        if inspectionFailDropOffInfo is not None:
            taskparameters['inspectionFailDropOffInfo'] = inspectionFailDropOffInfo
        if initialDetectionValidationInfo is not None:
            taskparameters['initialDetectionValidationInfo'] = initialDetectionValidationInfo
        if initialMoveRobotOufOfCameraOcclusion is not None:
            taskparameters['initialMoveRobotOufOfCameraOcclusion'] = initialMoveRobotOufOfCameraOcclusion
        if inputPlacedPartInfoOnArrivals is not None:
            taskparameters['inputPlacedPartInfoOnArrivals'] = inputPlacedPartInfoOnArrivals
        if ioSignalsInfo is not None:
            taskparameters['ioSignalsInfo'] = ioSignalsInfo
        if isGripperSyncExecPossible is not None:
            taskparameters['isGripperSyncExecPossible'] = isGripperSyncExecPossible
        if isTestMode is not None:
            taskparameters['isTestMode'] = isTestMode
        if isStopOnGripperPositionNotReached is not None:
            taskparameters['isStopOnGripperPositionNotReached'] = isStopOnGripperPositionNotReached
        if isStopOnObjectMassPropertiesMismatch is not None:
            taskparameters['isStopOnObjectMassPropertiesMismatch'] = isStopOnObjectMassPropertiesMismatch
        if isStopOnPieceLost is not None:
            taskparameters['isStopOnPieceLost'] = isStopOnPieceLost
        if isStopOnRobotExecutionError is not None:
            taskparameters['isStopOnRobotExecutionError'] = isStopOnRobotExecutionError
        if isStopOnTorqueLimitsError is not None:
            taskparameters['isStopOnTorqueLimitsError'] = isStopOnTorqueLimitsError
        if isStopOnTorqueLimitsErrorOutsidePickLocation is not None:
            taskparameters['isStopOnTorqueLimitsErrorOutsidePickLocation'] = isStopOnTorqueLimitsErrorOutsidePickLocation
        if isStopOnControllerError is not None:
            taskparameters['isStopOnControllerError'] = isStopOnControllerError
        if itlProgramNamesOnEvent is not None:
            taskparameters['itlProgramNamesOnEvent'] = itlProgramNamesOnEvent
        if cyclePreconditionIOInfo is not None:
            taskparameters['cyclePreconditionIOInfo'] = cyclePreconditionIOInfo
        if logmessagesmask is not None:
            taskparameters['logmessagesmask'] = logmessagesmask
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if justInTimeToolChangePlanning is not None:
            taskparameters['justInTimeToolChangePlanning'] = justInTimeToolChangePlanning
        if labelPlacingInfo is not None:
            taskparameters['labelPlacingInfo'] = labelPlacingInfo
        if labelerDirection is not None:
            taskparameters['labelerDirection'] = labelerDirection
        if localTargetDir is not None:
            taskparameters['localTargetDir'] = localTargetDir
        if logFailedTargetTimeout is not None:
            taskparameters['logFailedTargetTimeout'] = logFailedTargetTimeout
        if logFailedTargetPriorityTimeout is not None:
            taskparameters['logFailedTargetPriorityTimeout'] = logFailedTargetPriorityTimeout
        if maxAllowedTargetSize is not None:
            taskparameters['maxAllowedTargetSize'] = maxAllowedTargetSize
        if maxAllowedTargetSizeObjectName is not None:
            taskparameters['maxAllowedTargetSizeObjectName'] = maxAllowedTargetSizeObjectName
        if maxConsideredCameraIkSolutions is not None:
            taskparameters['maxConsideredCameraIkSolutions'] = maxConsideredCameraIkSolutions
        if maxDestIkSolutions is not None:
            taskparameters['maxDestIkSolutions'] = maxDestIkSolutions
        if maxGraspIkSolutions is not None:
            taskparameters['maxGraspIkSolutions'] = maxGraspIkSolutions
        if maxGraspsToConsider is not None:
            taskparameters['maxGraspsToConsider'] = maxGraspsToConsider
        if maxFinalPlanIgnoreCount is not None:
            taskparameters['maxFinalPlanIgnoreCount'] = maxFinalPlanIgnoreCount
        if maxFinalPlanIgnoreMinTargets is not None:
            taskparameters['maxFinalPlanIgnoreMinTargets'] = maxFinalPlanIgnoreMinTargets
        if maxGraspIkSolutionsPerGrasp is not None:
            taskparameters['maxGraspIkSolutionsPerGrasp'] = maxGraspIkSolutionsPerGrasp
        if maxIncidenceAngleOfIgnoreDestLinksForDestApproach is not None:
            taskparameters['maxIncidenceAngleOfIgnoreDestLinksForDestApproach'] = maxIncidenceAngleOfIgnoreDestLinksForDestApproach
        if maxIncidenceAngleOfIgnoreDestLinksForDestApproachIgnoreDistance is not None:
            taskparameters['maxIncidenceAngleOfIgnoreDestLinksForDestApproachIgnoreDistance'] = maxIncidenceAngleOfIgnoreDestLinksForDestApproachIgnoreDistance
        if maxStartFailuresForTargetGrasp is not None:
            taskparameters['maxStartFailuresForTargetGrasp'] = maxStartFailuresForTargetGrasp
        if maxStartFailuresForTarget is not None:
            taskparameters['maxStartFailuresForTarget'] = maxStartFailuresForTarget
        if maxLinearFailuresForTarget is not None:
            taskparameters['maxLinearFailuresForTarget'] = maxLinearFailuresForTarget
        if maxLinearFailuresForTargetGrasp is not None:
            taskparameters['maxLinearFailuresForTargetGrasp'] = maxLinearFailuresForTargetGrasp
        if maxDestFailuresForTargetGrasp is not None:
            taskparameters['maxDestFailuresForTargetGrasp'] = maxDestFailuresForTargetGrasp
        if maxTransferFailuresForTarget is not None:
            taskparameters['maxTransferFailuresForTarget'] = maxTransferFailuresForTarget
        if maxNumConsecutivePieceLost is not None:
            taskparameters['maxNumConsecutivePieceLost'] = maxNumConsecutivePieceLost
        if maxNumConsecutiveCycleFailures is not None:
            taskparameters['maxNumConsecutiveCycleFailures'] = maxNumConsecutiveCycleFailures
        if maxNumConsecutiveDistanceSensorFailures is not None:
            taskparameters['maxNumConsecutiveDistanceSensorFailures'] = maxNumConsecutiveDistanceSensorFailures
        if maxNumConsecutiveVerificationFailures is not None:
            taskparameters['maxNumConsecutiveVerificationFailures'] = maxNumConsecutiveVerificationFailures
        if maxNumConsecutiveRegistrationFailures is not None:
            taskparameters['maxNumConsecutiveRegistrationFailures'] = maxNumConsecutiveRegistrationFailures
        if maxNumConsecutiveSplitterOverweightFailures is not None:
            taskparameters['maxNumConsecutiveSplitterOverweightFailures'] = maxNumConsecutiveSplitterOverweightFailures
        if maxNumPlanningFailedIterations is not None:
            taskparameters['maxNumPlanningFailedIterations'] = maxNumPlanningFailedIterations
        if maxPlanningCyclesToQueue is not None:
            taskparameters['maxPlanningCyclesToQueue'] = maxPlanningCyclesToQueue
        if maxTimeForDecrease is not None:
            taskparameters['maxTimeForDecrease'] = maxTimeForDecrease
        if maxTorqueMultForApproach is not None:
            taskparameters['maxTorqueMultForApproach'] = maxTorqueMultForApproach
        if maxTorqueMultForDestApproach is not None:
            taskparameters['maxTorqueMultForDestApproach'] = maxTorqueMultForDestApproach
        if mergeTrajectoryParametersForRecovery is not None:
            taskparameters['mergeTrajectoryParametersForRecovery'] = mergeTrajectoryParametersForRecovery
        if midDestCoordType is not None:
            taskparameters['midDestCoordType'] = midDestCoordType
        if midDestIkparamNames is not None:
            taskparameters['midDestIkparamNames'] = midDestIkparamNames
        if midDestPassthroughVelocity is not None:
            taskparameters['midDestPassthroughVelocity'] = midDestPassthroughVelocity
        if midDestWaitTime is not None:
            taskparameters['midDestWaitTime'] = midDestWaitTime
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxManipSpeed is not None:
            taskparameters['maxManipSpeed'] = maxManipSpeed
        if maxnotifygrasps is not None:
            taskparameters['maxnotifygrasps'] = maxnotifygrasps
        if maxnotifydests is not None:
            taskparameters['maxnotifydests'] = maxnotifydests
        if maxTargetNeighborsOnFace is not None:
            taskparameters['maxTargetNeighborsOnFace'] = maxTargetNeighborsOnFace
        if minGraspDepartCompleteRatio is not None:
            taskparameters['minGraspDepartCompleteRatio'] = minGraspDepartCompleteRatio
        if minNumSameConsecutiveDetections is not None:
            taskparameters['minNumSameConsecutiveDetections'] = minNumSameConsecutiveDetections
        if minViableRegionPlanParameters is not None:
            taskparameters['minViableRegionPlanParameters'] = minViableRegionPlanParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if moveStraightParamsOnReplan is not None:
            taskparameters['moveStraightParamsOnReplan'] = moveStraightParamsOnReplan
        if moveToMidDest is not None:
            taskparameters['moveToMidDest'] = moveToMidDest
        if multiPickInfo is not None:
            taskparameters['multiPickInfo'] = multiPickInfo
        if neighTargetThresh is not None:
            taskparameters['neighTargetThresh'] = neighTargetThresh
        if neighTargetThreshForTorqueError is not None:
            taskparameters['neighTargetThreshForTorqueError'] = neighTargetThreshForTorqueError
        if neighTargetThreshY is not None:
            taskparameters['neighTargetThreshY'] = neighTargetThreshY
        if notifyStateSlaveZMQUri is not None:
            taskparameters['notifyStateSlaveZMQUri'] = notifyStateSlaveZMQUri
        if numInitialDestinationResultsWorkers is not None:
            taskparameters['numInitialDestinationResultsWorkers'] = numInitialDestinationResultsWorkers
        if numInitialGraspWorkers is not None:
            taskparameters['numInitialGraspWorkers'] = numInitialGraspWorkers
        if numInputPlacedPartInfoOnArrival is not None:
            taskparameters['numInputPlacedPartInfoOnArrival'] = numInputPlacedPartInfoOnArrival
        if numLogGraspPriorities is not None:
            taskparameters['numLogGraspPriorities'] = numLogGraspPriorities
        if numTargetsToPlace is not None:
            taskparameters['numTargetsToPlace'] = numTargetsToPlace
        if numThreads is not None:
            taskparameters['numThreads'] = numThreads
        if numValidBodiesToTriggerWaitForCapturingBeforeGraspApproach is not None:
            taskparameters['numValidBodiesToTriggerWaitForCapturingBeforeGraspApproach'] = numValidBodiesToTriggerWaitForCapturingBeforeGraspApproach
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if objectMassPropertiesCheckingInfo is not None:
            taskparameters['objectMassPropertiesCheckingInfo'] = objectMassPropertiesCheckingInfo
        if objectTypeDynamicParametersMap is not None:
            taskparameters['objectTypeDynamicParametersMap'] = objectTypeDynamicParametersMap
        if targetOverlapConstraintInfo is not None:
            taskparameters['targetOverlapConstraintInfo'] = targetOverlapConstraintInfo
        if packFormationComputationResult is not None:
            taskparameters['packFormationComputationResult'] = packFormationComputationResult
        if packFormationParameters is not None:
            taskparameters['packFormationParameters'] = packFormationParameters
        if orderPackFormationName is not None:
            taskparameters['orderPackFormationName'] = orderPackFormationName
        if packLocationName is not None:
            taskparameters['packLocationName'] = packLocationName
        if passOnDropAtDestinationNames is not None:
            taskparameters['passOnDropAtDestinationNames'] = passOnDropAtDestinationNames
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if pickFailureDepartRetryNum is not None:
            taskparameters['pickFailureDepartRetryNum'] = pickFailureDepartRetryNum
        if pickFailureDepartRetryWidth is not None:
            taskparameters['pickFailureDepartRetryWidth'] = pickFailureDepartRetryWidth
        if pickLocationInfo is not None:
            taskparameters['pickLocationInfo'] = pickLocationInfo
        if pieceInspectionInfo is not None:
            taskparameters['pieceInspectionInfo'] = pieceInspectionInfo
        if placedTargetPrefix is not None:
            taskparameters['placedTargetPrefix'] = placedTargetPrefix
        if placedTargetExtraPadding is not None:
            taskparameters['placedTargetExtraPadding'] = placedTargetExtraPadding
        if placedTargetRemainingTranslationRange is not None:
            taskparameters['placedTargetRemainingTranslationRange'] = placedTargetRemainingTranslationRange
        if placeLocationInfos is not None:
            taskparameters['placeLocationInfos'] = placeLocationInfos
        if planningSchedulingMode is not None:
            taskparameters['planningSchedulingMode'] = planningSchedulingMode
        if planningSmallestObjectSizeForCollision is not None:
            taskparameters['planningSmallestObjectSizeForCollision'] = planningSmallestObjectSizeForCollision
        if planToDestMode is not None:
            taskparameters['planToDestMode'] = planToDestMode
        if predictDetectionInfo is not None:
            taskparameters['predictDetectionInfo'] = predictDetectionInfo
        if postCycleExecution is not None:
            taskparameters['postCycleExecution'] = postCycleExecution
        if putBackParameters is not None:
            taskparameters['putBackParameters'] = putBackParameters
        if randomBoxInfo is not None:
            taskparameters['randomBoxInfo'] = randomBoxInfo
        if recoverySpeedMult is not None:
            taskparameters['recoverySpeedMult'] = recoverySpeedMult
        if rejectGraspOverlappingNonPickable is not None:
            taskparameters['rejectGraspOverlappingNonPickable'] = rejectGraspOverlappingNonPickable
        if restrictLabelOrientationInfo is not None:
            taskparameters['restrictLabelOrientationInfo'] = restrictLabelOrientationInfo
        if robotCycleStartPosition is not None:
            taskparameters['robotCycleStartPosition'] = robotCycleStartPosition
        if robotFinishPosition is not None:
            taskparameters['robotFinishPosition'] = robotFinishPosition
        if robotRecoveryPosition is not None:
            taskparameters['robotRecoveryPosition'] = robotRecoveryPosition
        if robotRecoveryDepartAccel is not None:
            taskparameters['robotRecoveryDepartAccel'] = robotRecoveryDepartAccel
        if robotRecoveryDepartSpeed is not None:
            taskparameters['robotRecoveryDepartSpeed'] = robotRecoveryDepartSpeed
        if robotRecoveryDepartOffsetDir is not None:
            taskparameters['robotRecoveryDepartOffsetDir'] = robotRecoveryDepartOffsetDir
        if robotRecoveryDepartOffsetInTool is not None:
            taskparameters['robotRecoveryDepartOffsetInTool'] = robotRecoveryDepartOffsetInTool
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveEnvState is not None:
            taskparameters['saveEnvState'] = saveEnvState
        if saveEnvStateOnUnexpectedFinish is not None:
            taskparameters['saveEnvStateOnUnexpectedFinish'] = saveEnvStateOnUnexpectedFinish
        if saveGrabRecoveryScene is not None:
            taskparameters['saveGrabRecoveryScene'] = saveGrabRecoveryScene
        if savePlanningDiagnosticData is not None:
            taskparameters['savePlanningDiagnosticData'] = savePlanningDiagnosticData
        if saveRecoveryScene is not None:
            taskparameters['saveRecoveryScene'] = saveRecoveryScene
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if saveOnPlanFailure is not None:
            taskparameters['saveOnPlanFailure'] = saveOnPlanFailure
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if saveVerificationScene is not None:
            taskparameters['saveVerificationScene'] = saveVerificationScene
        if saveWhenSlowPlanningDuration is not None:
            taskparameters['saveWhenSlowPlanningDuration'] = saveWhenSlowPlanningDuration
        if timeOutForFailedParts is not None:
            taskparameters['timeOutForFailedParts'] = timeOutForFailedParts
        if useKZFilter is not None:
            taskparameters['useKZFilter'] = useKZFilter
        if singularityEpsilon is not None:
            taskparameters['singularityEpsilon'] = singularityEpsilon
        if skipCollidingDestsInfo is not None:
            taskparameters['skipCollidingDestsInfo'] = skipCollidingDestsInfo
        if skipLastImageCheckWhenNoMoreDestForDynamicGoals is not None:
            taskparameters['skipLastImageCheckWhenNoMoreDestForDynamicGoals'] = skipLastImageCheckWhenNoMoreDestForDynamicGoals
        if skipTrajectoryPlanning is not None:
            taskparameters['skipTrajectoryPlanning'] = skipTrajectoryPlanning
        if splitterSheetInfo is not None:
            taskparameters['splitterSheetInfo'] = splitterSheetInfo
        if smootherParameters is not None:
            taskparameters['smootherParameters'] = smootherParameters
        if sourceDynamicGoalsGeneratorParametersOverwrite is not None:
            taskparameters['sourceDynamicGoalsGeneratorParametersOverwrite'] = sourceDynamicGoalsGeneratorParametersOverwrite
        if sourcecameranames is not None:
            taskparameters['sourcecameranames'] = sourcecameranames
        if sourcecontainername is not None:
            taskparameters['sourcecontainername'] = sourcecontainername
        if sourceDestTargetOrientationPenalty is not None:
            taskparameters['sourceDestTargetOrientationPenalty'] = sourceDestTargetOrientationPenalty
        if strictPickOrdering is not None:
            taskparameters['strictPickOrdering'] = strictPickOrdering
        if targetdestikthresh is not None:
            taskparameters['targetdestikthresh'] = targetdestikthresh
        if targetenvclearance is not None:
            taskparameters['targetenvclearance'] = targetenvclearance
        if targetGraspTimeLimit is not None:
            taskparameters['targetGraspTimeLimit'] = targetGraspTimeLimit
        if targetIsRandomBox is not None:
            taskparameters['targetIsRandomBox'] = targetIsRandomBox
        if targetMinBottomPaddingForInitialTransfer is not None:
            taskparameters['targetMinBottomPaddingForInitialTransfer'] = targetMinBottomPaddingForInitialTransfer
        if targetMinSafetyHeightForInitialTransfer is not None:
            taskparameters['targetMinSafetyHeightForInitialTransfer'] = targetMinSafetyHeightForInitialTransfer
        if targetPaddingForGrasping is not None:
            taskparameters['targetPaddingForGrasping'] = targetPaddingForGrasping
        if targetPlaceIkParamName is not None:
            taskparameters['targetPlaceIkParamName'] = targetPlaceIkParamName
        if targetPlaceTranslationOffset is not None:
            taskparameters['targetPlaceTranslationOffset'] = targetPlaceTranslationOffset
        if targetPlaceTranslationOffsetInDest is not None:
            taskparameters['targetPlaceTranslationOffsetInDest'] = targetPlaceTranslationOffsetInDest
        if targetPostActionAfterPick is not None:
            taskparameters['targetPostActionAfterPick'] = targetPostActionAfterPick
        if targetPriorityMultipliers is not None:
            taskparameters['targetPriorityMultipliers'] = targetPriorityMultipliers
        if targetStackDestHeightPenaltyMult is not None:
            taskparameters['targetStackDestHeightPenaltyMult'] = targetStackDestHeightPenaltyMult
        if targetPrioritySuctionForceMult is not None:
            taskparameters['targetPrioritySuctionForceMult'] = targetPrioritySuctionForceMult
        if targetPriorityTransferSpeedMult is not None:
            taskparameters['targetPriorityTransferSpeedMult'] = targetPriorityTransferSpeedMult
        if targeturi is not None:
            taskparameters['targeturi'] = targeturi
        if targetThicknessThreshForTrapping is not None:
            taskparameters['targetThicknessThreshForTrapping'] = targetThicknessThreshForTrapping
        if toolMaxRotationBetweenPickAndPlace is not None:
            taskparameters['toolMaxRotationBetweenPickAndPlace'] = toolMaxRotationBetweenPickAndPlace
        if targetRotationConstraintParameters is not None:
            taskparameters['targetRotationConstraintParameters'] = targetRotationConstraintParameters
        if targetLabelAlignmentParameters is not None:
            taskparameters['targetLabelAlignmentParameters'] = targetLabelAlignmentParameters
        if treatAsSquareTargetExtentsThreshold is not None:
            taskparameters['treatAsSquareTargetExtentsThreshold'] = treatAsSquareTargetExtentsThreshold
        if toolPosConstraintPaddingXYZ is not None:
            taskparameters['toolPosConstraintPaddingXYZ'] = toolPosConstraintPaddingXYZ
        if toolposes is not None:
            taskparameters['toolposes'] = toolposes
        if toolSpeedAccelInfo is not None:
            taskparameters['toolSpeedAccelInfo'] = toolSpeedAccelInfo
        if toolSpeedAccelOptions is not None:
            taskparameters['toolSpeedAccelOptions'] = toolSpeedAccelOptions
        if torqueDistThresh is not None:
            taskparameters['torqueDistThresh'] = torqueDistThresh
        if transferSpeedPostMult is not None:
            taskparameters['transferSpeedPostMult'] = transferSpeedPostMult
        if minAcceptedTransferSpeedMult is not None:
            taskparameters['minAcceptedTransferSpeedMult'] = minAcceptedTransferSpeedMult
        if minTransferSpeedMult is not None:
            taskparameters['minTransferSpeedMult'] = minTransferSpeedMult
        if transferSpeedMultPerWeight is not None:
            taskparameters['transferSpeedMultPerWeight'] = transferSpeedMultPerWeight
        if transferTrajectoryCostMultipliers is not None:
            taskparameters['transferTrajectoryCostMultipliers'] = transferTrajectoryCostMultipliers
        if transferTrajectoryCostFnDiscretizationStep is not None:
            taskparameters['transferTrajectoryCostFnDiscretizationStep'] = transferTrajectoryCostFnDiscretizationStep
        if updateHeightWithMeasuredValue is not None:
            taskparameters['updateHeightWithMeasuredValue'] = updateHeightWithMeasuredValue
        if updateMassWithMeasuredValue is not None:
            taskparameters['updateMassWithMeasuredValue'] = updateMassWithMeasuredValue
        if unitMass is not None:
            taskparameters['unitMass'] = unitMass
        if useBarcodeUnpickableRegions is not None:
            taskparameters['useBarcodeUnpickableRegions'] = useBarcodeUnpickableRegions
        if useDecreaseTargetEnvClearanceBigObjects is not None:
            taskparameters['useDecreaseTargetEnvClearanceBigObjects'] = useDecreaseTargetEnvClearanceBigObjects
        if useDetectedFace is not None:
            taskparameters['useDetectedFace'] = useDetectedFace
        if useDestGoalsInCollision is not None:
            taskparameters['useDestGoalsInCollision'] = useDestGoalsInCollision
        if useDropInSourceDests is not None:
            taskparameters['useDropInSourceDests'] = useDropInSourceDests
        if useDynamicGoals is not None:
            taskparameters['useDynamicGoals'] = useDynamicGoals
        if useExecutionQueueing is not None:
            taskparameters['useExecutionQueueing'] = useExecutionQueueing
        if useFailThresholds is not None:
            taskparameters['useFailThresholds'] = useFailThresholds
        if useLocationState is not None:
            taskparameters['useLocationState'] = useLocationState
        if waitDynamicGoalPointCloudTimeout is not None:
            taskparameters['waitDynamicGoalPointCloudTimeout'] = waitDynamicGoalPointCloudTimeout
        if validatePlacedAllPickable is not None:
            taskparameters['validatePlacedAllPickable'] = validatePlacedAllPickable
        if waitForBetterCostTime is not None:
            taskparameters['waitForBetterCostTime'] = waitForBetterCostTime
        if waitForSupplyTimeout is not None:
            taskparameters['waitForSupplyTimeout'] = waitForSupplyTimeout
        if waitForDetectionAfterInitialPick is not None:
            taskparameters['waitForDetectionAfterInitialPick'] = waitForDetectionAfterInitialPick
        if waitForDestProhibitedAtGrasp is not None:
            taskparameters['waitForDestProhibitedAtGrasp'] = waitForDestProhibitedAtGrasp
        if waitForSourceProhibitedAtDest is not None:
            taskparameters['waitForSourceProhibitedAtDest'] = waitForSourceProhibitedAtDest
        if waitForStateTrigger is not None:
            taskparameters['waitForStateTrigger'] = waitForStateTrigger
        if waitForLocationUnprohibited is not None:
            taskparameters['waitForLocationUnprohibited'] = waitForLocationUnprohibited
        if waitUpdateStampTimeout is not None:
            taskparameters['waitUpdateStampTimeout'] = waitUpdateStampTimeout
        if ykkControlInfo is not None:
            taskparameters['ykkControlInfo'] = ykkControlInfo
        if controllerclientparameters is not None:
            taskparameters['controllerclientparameters'] = controllerclientparameters
        if cycleIndex is not None:
            taskparameters['cycleIndex'] = cycleIndex
        if cycleStartUseToolPose != False:
            taskparameters['cycleStartUseToolPose'] = cycleStartUseToolPose
        if destGoals is not None:
            taskparameters['destGoals'] = destGoals
        if destcontainernames is not None:
            taskparameters['destcontainernames'] = destcontainernames
        if destSensorSelectionInfos is not None:
            taskparameters['destSensorSelectionInfos'] = destSensorSelectionInfos
        if detectionInfos is not None:
            taskparameters['detectionInfos'] = detectionInfos
        if forceMoveToFinish is not None:
            taskparameters['forceMoveToFinish'] = forceMoveToFinish
        if forceStartRobotPositionConfigurationName is not None:
            taskparameters['forceStartRobotPositionConfigurationName'] = forceStartRobotPositionConfigurationName
        if ignoreStartPosition is not None:
            taskparameters['ignoreStartPosition'] = ignoreStartPosition
        if initiallyDisableRobotBridge is not None:
            taskparameters['initiallyDisableRobotBridge'] = initiallyDisableRobotBridge
        if inputPartIndex is not None:
            taskparameters['inputPartIndex'] = inputPartIndex
        if itlParameters is not None:
            taskparameters['itlParameters'] = itlParameters
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if pickContainerHasOnlyOnePart != False:
            taskparameters['pickContainerHasOnlyOnePart'] = pickContainerHasOnlyOnePart
        if placedTargetPrefixes is not None:
            taskparameters['placedTargetPrefixes'] = placedTargetPrefixes
        if registrationInfo is not None:
            taskparameters['registrationInfo'] = registrationInfo
        if sourcecontainernames is not None:
            taskparameters['sourcecontainernames'] = sourcecontainernames
        if sourceSensorSelectionInfos is not None:
            taskparameters['sourceSensorSelectionInfos'] = sourceSensorSelectionInfos
        if targetname is not None:
            taskparameters['targetname'] = targetname
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, robotspeed=robotspeed, timeout=timeout)

    def StopPickPlaceThread(self, resetExecutionState=_deprecated, resetStatusPickPlace=False, finishStatus=None, finishMessage=None, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, resetCachedRobotConfigurationState=None, useRobotBridge=None, **kwargs):
        # type: (bool, bool, str, Optional[str], float, bool, Optional[types.StopPickPlaceThreadParametersDynamicEnvironmentState], Optional[int], Optional[bool], Optional[bool], Optional[Any]) -> Optional[Any]
        """
        stops the pick and place thread started with StartPickAndPlaceThread

        Args:
            resetExecutionState: **deprecated** If True, then reset the order state variables. By default True. (Default: True)
            resetStatusPickPlace: If True, then reset the statusPickPlace field of hte planning slave. (Default: False)
            finishStatus: Optional finish code to end the cycle with (if it doesn't end with something else beforehand). (Default: None)
            finishMessage: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            resetCachedRobotConfigurationState: If True, then reset the cached robot configuration state. By default False. (Default: None)
            useRobotBridge: (Default: None)
        """
        taskparameters = {
            'command': 'StopPickPlaceThread',
            'resetStatusPickPlace': resetStatusPickPlace,
            'finishStatus': finishStatus,
        }  # type: dict[str, Any]
        if finishMessage is not None:
            taskparameters['finishMessage'] = finishMessage
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if resetCachedRobotConfigurationState is not None:
            taskparameters['resetCachedRobotConfigurationState'] = resetCachedRobotConfigurationState
        if useRobotBridge is not None:
            taskparameters['useRobotBridge'] = useRobotBridge
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetPickPlaceStatus(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.GetPickPlaceStatusParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.GetPickPlaceStatusReturns]
        """
        Gets the status of the pick and place thread

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            Status of the pick and place thread.
        """
        taskparameters = {
            'command': 'GetPickPlaceStatus',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ComputeIK(self, toolname=None, timeout=10, iktype=None, quaternion=None, translation=None, direction=None, angle=None, freeincvalue=None, filteroptions=None, limit=None, preshape=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Optional[str], float, Optional[str], Optional[list[float]], Optional[list[float]], Optional[list[Any]], Optional[float], Optional[float], Optional[int], Optional[int], Optional[list[float]], Optional[types.ComputeIKParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.ComputeIKReturns]
        """
        Args:
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            iktype: grasp (but basically the just the ikparam) (Default: None)
            quaternion: List specifying the quaternion in w,x,y,z format, e.g. [1,0,0,0]. (Default: None)
            translation: List of x,y,z values of the object in millimeters. (Default: None)
            direction: grasp (but basically the just the ikparam) direction in world cooordinates (Default: None)
            angle: grasp (but basically the just the ikparam) angle in world cooordinates (Default: None)
            freeincvalue: The discretization of the free joints of the robot when computing ik. (Default: None)
            filteroptions: OpenRAVE IkFilterOptions bitmask. By default this is 1, which means all collisions are checked (Default: None)
            limit: number of solutions to return (Default: None)
            preshape: If the tool has fingers after the end effector, specify their values. The gripper DOFs come from **gripper_dof_pks** field from the tool. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ComputeIK',
        }  # type: dict[str, Any]
        if iktype is not None:
            taskparameters['iktype'] = iktype
        if quaternion is not None:
            taskparameters['quaternion'] = quaternion
        if translation is not None:
            taskparameters['translation'] = translation
        if direction is not None:
            taskparameters['direction'] = direction
        if angle is not None:
            taskparameters['angle'] = angle
        if freeincvalue is not None:
            taskparameters['freeincvalue'] = freeincvalue
        if filteroptions is not None:
            taskparameters['filteroptions'] = filteroptions
        if limit is not None:
            taskparameters['limit'] = limit
        if preshape is not None:
            taskparameters['preshape'] = preshape
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def InitializePartsWithPhysics(self, timeout=10, targeturi=None, numtargets=None, regionname=None, duration=None, basename=None, deleteprevious=None, forcegravity=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[Any], Optional[Any], Optional[str], Optional[Any], Optional[Any], Optional[Any], Optional[Any], Optional[types.InitializePartsWithPhysicsParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Start a physics simulation where the parts drop down into the bin. The method returns as soon as the physics is initialized, user has to wait for the "duration" or call StopPhysicsThread command.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            targeturi: the target uri to initialize the scene with (Default: None)
            numtargets: the number of targets to create (Default: None)
            regionname: The container name to drop the parts into. (Default: None)
            duration: the duration in seconds to continue the physics until it is stopped. (Default: None)
            basename: The basename to give to all the new target names. Numbers are suffixed at the end, like basename+'0134'. If not specified, will use a basename derived from the targeturi. (Default: None)
            deleteprevious: if True, will delete all the previous targets in the scene. By default this is True. (Default: None)
            forcegravity: if not None, the gravity with which the objects should fall with. If None, then uses the scene's gravity (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'InitializePartsWithPhysics',
        }  # type: dict[str, Any]
        if targeturi is not None:
            taskparameters['targeturi'] = targeturi
        if numtargets is not None:
            taskparameters['numtargets'] = numtargets
        if regionname is not None:
            taskparameters['containername'] = regionname
        if duration is not None:
            taskparameters['duration'] = duration
        if basename is not None:
            taskparameters['basename'] = basename
        if deleteprevious is not None:
            taskparameters['deleteprevious'] = deleteprevious
        if forcegravity is not None:
            taskparameters['forcegravity'] = forcegravity
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        if 'containername' not in taskparameters:
            taskparameters['containername'] = self.regionname
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def StopPhysicsThread(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.StopPhysicsThreadParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        stops the physics simulation started with InitializePartsWithPhysics

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'StopPhysicsThread',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def JitterPartUntilValidGrasp(self, toolname=None, timeout=10, targetname=None, graspsetname=None, approachoffset=None, departoffsetdir=None, destdepartoffsetdir=None, leaveoffsetintool=None, desttargetname=None, destikparamnames=None, jitterangle=None, jitteriters=None, dynamicEnvironmentState=None, debuglevel=None, jitterdist=None, **kwargs):
        # type: (Optional[str], float, Optional[str], Optional[str], Optional[Any], Optional[tuple[float, float, float]], Optional[tuple[float, float, float]], Optional[Any], Optional[Any], Optional[list[list[str]]], Optional[Any], Optional[Any], Optional[types.JitterPartUntilValidGraspParametersDynamicEnvironmentState], Optional[int], Optional[Any], Optional[Any]) -> Optional[Any]
        """
        Select a part that wasn't able to be grasped and jitter its location such that a grasp set is found for it that will take it to the destination.

        Args:
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            targetname: Name of the target object. (Default: None)
            graspsetname: Name of the grasp set to use (Default: None)
            approachoffset: The approach distance for simulating full grasp. (Default: None)
            departoffsetdir: The depart distance for simulating full grasp. (Default: None)
            destdepartoffsetdir: The direction and distance in mm to move away from the object after it is placed, e.g. [0,0,30]. Depending on leaveoffsetintool parameter, this can in the global coordinate system or tool coordinate system. (Default: None)
            leaveoffsetintool: If 1, destdepartoffsetdir is in the tool coordinate system. If 0, destdepartoffsetdir is in the global coordinate system. By default this is 0. (Default: None)
            desttargetname: The destination target name where the destination goal ikparams come from. If no name is specified, then robot won't consider putting the target into the destination when it searches for grasps. (Default: None)
            destikparamnames: A list of lists of ikparam names for the ordered destinations of the target. destikparamnames[0] is where the first picked up part goes, desttargetname[1] is where the second picked up target goes. (Default: None)
            jitterangle: Amount to jitter the target object's orientation angle (Default: None)
            jitteriters: Number of times to try jittering before giving up. (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            jitterdist: Amount to jitter the target object translation by (Default: None)

        Returns:
            If failed, an empty dictionary. If succeeded, a dictionary with the following keys:
            - translation: the new translation of the target part
            - quaternion: the new quaternion of the target part
            - jointvalues: robot joint values that are grasping the part (fingers are at their preshape).
            - graspname: the grasp name used for jointvalues. If empty, then no grasp was found.
            - destikname: the name of the destination ikparam that was chosen with the grasp
            - destjointvalues: robot joint values at one of the specified destinations (fingers are at their final positions).
            - desttranslation: the new translation of the target part
            - destquaternion: the new quaternion of the target part
        """
        taskparameters = {
            'command': 'JitterPartUntilValidGrasp',
        }  # type: dict[str, Any]
        if targetname is not None:
            taskparameters['targetname'] = targetname
        if graspsetname is not None:
            taskparameters['graspsetname'] = graspsetname
        if approachoffset is not None:
            taskparameters['approachoffset'] = approachoffset
        if departoffsetdir is not None:
            taskparameters['departoffsetdir'] = departoffsetdir
        if destdepartoffsetdir is not None:
            taskparameters['destdepartoffsetdir'] = destdepartoffsetdir
        if leaveoffsetintool is not None:
            taskparameters['leaveoffsetintool'] = leaveoffsetintool
        if desttargetname is not None:
            taskparameters['desttargetname'] = desttargetname
        if destikparamnames is not None:
            taskparameters['destikparamnames'] = destikparamnames
        if jitterangle is not None:
            taskparameters['jitterangle'] = jitterangle
        if jitteriters is not None:
            taskparameters['jitteriters'] = jitteriters
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if jitterdist is not None:
            taskparameters['jitterdist'] = jitterdist
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, toolname=toolname, timeout=timeout)

    def MoveToDropOff(
        self,
        dropOffInfo,  # type: types.MoveToDropOffParametersDropOffInfo
        robotname=None,  # type: Optional[str]
        robotspeed=None,  # type: Optional[float]
        robotaccelmult=None,  # type: Optional[float]
        execute=1,  # type: int
        startvalues=None,  # type: Optional[list[float]]
        envclearance=None,  # type: Optional[float]
        timeout=10,  # type: float
        dynamicEnvironmentState=None,  # type: Optional[types.MoveToDropOffParametersDynamicEnvironmentState]
        debuglevel=None,  # type: Optional[int]
        unit='mm',  # type: str
        constraintToolDirection=None,  # type: Optional[tuple[float, float, float, float, float, float, float]]
        departOffsetDir=None,  # type: Optional[tuple[float, float, float]]
        departMinimumCompleteRatio=None,  # type: Optional[float]
        departOffsetAwayFromGravity=None,  # type: Optional[float]
        trajname=None,  # type: Optional[str]
        disablebodies=None,  # type: Optional[bool]
        ignoreGrabbingTarget=None,  # type: Optional[bool]
        jointthresh=None,  # type: Optional[float]
        jitter=None,  # type: Optional[float]
        executionFilterFactor=None,  # type: Optional[float]
        filtertraj=None,  # type: Optional[bool]
        locationCollisionInfos=None,  # type: Optional[list[types.MoveToDropOffParametersLocationCollisionInfosArrayElement]]
        currentlimitratios=None,  # type: Optional[list[float]]
        goalJointThreshold=None,  # type: Optional[list[float]]
        goalWorkspaceThreshold=None,  # type: Optional[float]
        calibrategripper=None,  # type: Optional[bool]
        departAccel=None,  # type: Optional[float]
        departOffsetInTool=None,  # type: Optional[bool]
        departSpeed=None,  # type: Optional[float]
        maxManipAccel=None,  # type: Optional[float]
        maxJitterLinkDist=None,  # type: Optional[float]
        pathPlannerParameters=None,  # type: Optional[types.PathPlannerParameters]
        moveStraightParams=None,  # type: Optional[types.MoveStraightParameters]
        forceTorqueBasedEstimatorParameters=None,  # type: Optional[types.ForceTorqueBasedEstimatorParameters]
        savetrajectorylog=None,  # type: Optional[bool]
        saveRobotFeedbackLog=None,  # type: Optional[bool]
        loadRobotFeedbackLog=None,  # type: Optional[bool]
        saveConcatenateTrajectoryLog=None,  # type: Optional[bool]
        saveFilterTrajectoryLog=None,  # type: Optional[bool]
        executionConnectingTrajReverseMult=None,  # type: Optional[float]
        executionReverseRecoveryDistance=None,  # type: Optional[float]
        jittererParameters=None,  # type: Optional[types.JittererParameters]
        gripperInfo=None,  # type: Optional[types.MoveToDropOffParametersGripperInfoVariantItemPrefix0]
        speed=_deprecated,  # type: Optional[Any]
        ionames=None,  # type: Optional[list[Any]]
        positionConfigurationName=None,  # type: Optional[str]
        positionConfigurationCandidateNames=None,  # type: Optional[list[str]]
        toolname=None,  # type: Optional[str]
        robotspeedmult=None,  # type: Optional[float]
        robotJointNames=None,  # type: Optional[list[str]]
        jointindices=_deprecated,  # type: Optional[list[int]]
        startJointConfigurationStates=None,  # type: Optional[list[types.MoveToDropOffParametersStartJointConfigurationStatesArrayElement]]
        goalJointConfigurationStates=None,  # type: Optional[list[types.MoveToDropOffParametersGoalJointConfigurationStatesArrayElement]]
        goaljoints=None,  # type: Optional[list[float]]
        robotBridgeConnectionInfo=None,  # type: Optional[types.MoveToDropOffParametersRobotBridgeConnectionInfo]
        useReleaseTargetData=None,  # type: Optional[bool]
        clearOrderCache=None,  # type: Optional[bool]
        **kwargs  # type: Optional[Any]
    ):
        # type: (...) -> Optional[Any]
        """
        Moves the robot to desired joint angles.

        Args:
            dropOffInfo: Information used to drop off at a location
            robotname: Name of the robot (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotaccelmult: Value in (0,1] defining the percentage of acceleration the robot should move at. (Default: None)
            execute: If 1, execute the motion. (Default: 1)
            startvalues: The robot joint values to start the motion from. (Default: None)
            envclearance: Environment clearance in millimeters. (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            constraintToolDirection: Contains 7 params: manipdir, globaldir, cosangle. (Default: None)
            departOffsetDir: Direction in which to apply the offset when departing from the pick/place operation. (Default: None)
            departMinimumCompleteRatio: The ratio of the linear depart motion that needs to be possible for a pick/place to be executed. Pick/place candidate locations that do not allow sufficient space for the depart motion are discarded. Generally between 0.0 and 1.0. (Default: None)
            departOffsetAwayFromGravity: The distance to depart vertically upwards after picking/placing. Overridden by departOffsetDir. (Default: None)
            trajname: (Default: None)
            disablebodies: (Default: None)
            ignoreGrabbingTarget: (Default: None)
            jointthresh: (Default: None)
            jitter: (Default: None)
            executionFilterFactor: (Default: None)
            filtertraj: (Default: None)
            locationCollisionInfos: List of external collision IOs to be computed and sent in realtime. (Default: None)
            currentlimitratios: The joints' current limit ratios. (Default: None)
            goalJointThreshold: Threshold of the sum of abs joint differences between what the robot is able to achieve and where the goal is, in degrees. If not within this threshold, robot tries to reach goal, during some time. (Default: None)
            goalWorkspaceThreshold: Threshold in mm. If the robot manipulator is within this threshold to the goal position, then trajectory is assumed to be successful. (Default: None)
            calibrategripper: (Default: None)
            departAccel: (Default: None)
            departOffsetInTool: (Default: None)
            departSpeed: (Default: None)
            maxManipAccel: (Default: None)
            maxJitterLinkDist: mm, When jittering, the max distance any link on the robot can move. (Default: None)
            pathPlannerParameters: Parameters for robot path planning. (Default: None)
            moveStraightParams: A set of parameters defining how the robot behaves during linear motions. (Default: None)
            forceTorqueBasedEstimatorParameters: Parameters for state estimation features based on force torque sensor (Default: None)
            savetrajectorylog: If True, will save the commanded (input) trajectories before they are executed (Default: None)
            saveRobotFeedbackLog: If True, will tell robotbridge to save trajectory files (Default: None)
            loadRobotFeedbackLog: If True, will tell robotbridge to load the robot feedback log after trajectory ends (Default: None)
            saveConcatenateTrajectoryLog: If True, will save trajectories used for inputs of concatenate trajectory functions (Default: None)
            saveFilterTrajectoryLog: If True, will save trajectories used for filtering, such as SmartFilter (Default: None)
            executionConnectingTrajReverseMult: Used for several code paths such as MoveToolLinear, MoveJointsNoDec, MoveToHandPosition. This is passed to robotbridge. If None, robotbridge uses default value internally. (Default: None)
            executionReverseRecoveryDistance: Specifies the reversing distance for trajectories to recover from collision/position error. This is passed to robotbridge. If None, robotbridge uses default internally (most likely 50 mm). (Default: None)
            jittererParameters: Parameters dealing with jittering the robot out of collisions. (Default: None)
            gripperInfo: TODO(felixvd): Check if this really propagates. (Default: None)
            speed: **deprecated** Use robotspeed instead. (Default: None)
            ionames: A list of IO names to read/write (Default: None)
            positionConfigurationName: If specified, the name of position configuration to move to. If it does not exist, will raise an error. (Default: None)
            positionConfigurationCandidateNames: If specified, goes to the first position that is defined for the robot. If no positions exist, returns without moving the robot. (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            robotspeedmult: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            robotJointNames: (Default: None)
            jointindices: **deprecated** List of corresponding joint indices, default is range(len(jointvalues)) (Default: None)
            startJointConfigurationStates: List of dicts for each joint. (Default: None)
            goalJointConfigurationStates: List of dicts for each joint entry. (Default: None)
            goaljoints: List of joint values to move to. (Default: None)
            robotBridgeConnectionInfo: Information to set up a client to the robot bridge. (Default: None)
            useReleaseTargetData: (Default: None)
            clearOrderCache: (Default: None)
        """
        taskparameters = {
            'command': 'MoveToDropOff',
            'dropOffInfo': dropOffInfo,
            'unit': unit,
        }  # type: dict[str, Any]
        if execute != 1:
            taskparameters['execute'] = execute
        if startvalues is not None:
            taskparameters['startvalues'] = list(startvalues)
        if envclearance is not None:
            taskparameters['envclearance'] = envclearance
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if constraintToolDirection is not None:
            taskparameters['constraintToolDirection'] = constraintToolDirection
        if departOffsetDir is not None:
            taskparameters['departOffsetDir'] = departOffsetDir
        if departMinimumCompleteRatio is not None:
            taskparameters['departMinimumCompleteRatio'] = departMinimumCompleteRatio
        if departOffsetAwayFromGravity is not None:
            taskparameters['departOffsetAwayFromGravity'] = departOffsetAwayFromGravity
        if trajname is not None:
            taskparameters['trajname'] = trajname
        if disablebodies is not None:
            taskparameters['disablebodies'] = disablebodies
        if ignoreGrabbingTarget is not None:
            taskparameters['ignoreGrabbingTarget'] = ignoreGrabbingTarget
        if jointthresh is not None:
            taskparameters['jointthresh'] = jointthresh
        if jitter is not None:
            taskparameters['jitter'] = jitter
        if executionFilterFactor is not None:
            taskparameters['executionFilterFactor'] = executionFilterFactor
        if filtertraj is not None:
            taskparameters['filtertraj'] = filtertraj
        if locationCollisionInfos is not None:
            taskparameters['locationCollisionInfos'] = locationCollisionInfos
        if currentlimitratios is not None:
            taskparameters['currentlimitratios'] = currentlimitratios
        if goalJointThreshold is not None:
            taskparameters['goalJointThreshold'] = goalJointThreshold
        if goalWorkspaceThreshold is not None:
            taskparameters['goalWorkspaceThreshold'] = goalWorkspaceThreshold
        if calibrategripper is not None:
            taskparameters['calibrategripper'] = calibrategripper
        if departAccel is not None:
            taskparameters['departAccel'] = departAccel
        if departOffsetInTool is not None:
            taskparameters['departOffsetInTool'] = departOffsetInTool
        if departSpeed is not None:
            taskparameters['departSpeed'] = departSpeed
        if maxManipAccel is not None:
            taskparameters['maxManipAccel'] = maxManipAccel
        if maxJitterLinkDist is not None:
            taskparameters['maxJitterLinkDist'] = maxJitterLinkDist
        if pathPlannerParameters is not None:
            taskparameters['pathPlannerParameters'] = pathPlannerParameters
        if moveStraightParams is not None:
            taskparameters['moveStraightParams'] = moveStraightParams
        if forceTorqueBasedEstimatorParameters is not None:
            taskparameters['forceTorqueBasedEstimatorParameters'] = forceTorqueBasedEstimatorParameters
        if savetrajectorylog is not None:
            taskparameters['savetrajectorylog'] = savetrajectorylog
        if saveRobotFeedbackLog is not None:
            taskparameters['saveRobotFeedbackLog'] = saveRobotFeedbackLog
        if loadRobotFeedbackLog is not None:
            taskparameters['loadRobotFeedbackLog'] = loadRobotFeedbackLog
        if saveConcatenateTrajectoryLog is not None:
            taskparameters['saveConcatenateTrajectoryLog'] = saveConcatenateTrajectoryLog
        if saveFilterTrajectoryLog is not None:
            taskparameters['saveFilterTrajectoryLog'] = saveFilterTrajectoryLog
        if executionConnectingTrajReverseMult is not None:
            taskparameters['executionConnectingTrajReverseMult'] = executionConnectingTrajReverseMult
        if executionReverseRecoveryDistance is not None:
            taskparameters['executionReverseRecoveryDistance'] = executionReverseRecoveryDistance
        if jittererParameters is not None:
            taskparameters['jittererParameters'] = jittererParameters
        if gripperInfo is not None:
            taskparameters['gripperInfo'] = gripperInfo
        if ionames is not None:
            taskparameters['ionames'] = ionames
        if positionConfigurationName is not None:
            taskparameters['positionConfigurationName'] = positionConfigurationName
        if positionConfigurationCandidateNames is not None:
            taskparameters['positionConfigurationCandidateNames'] = positionConfigurationCandidateNames
        if toolname is not None:
            taskparameters['toolname'] = toolname
        if robotspeedmult is not None:
            taskparameters['robotspeedmult'] = robotspeedmult
        if robotJointNames is not None:
            taskparameters['robotJointNames'] = robotJointNames
        if startJointConfigurationStates is not None:
            taskparameters['startJointConfigurationStates'] = startJointConfigurationStates
        if goalJointConfigurationStates is not None:
            taskparameters['goalJointConfigurationStates'] = goalJointConfigurationStates
        if goaljoints is not None:
            taskparameters['goaljoints'] = goaljoints
        if robotBridgeConnectionInfo is not None:
            taskparameters['robotBridgeConnectionInfo'] = robotBridgeConnectionInfo
        if useReleaseTargetData is not None:
            taskparameters['useReleaseTargetData'] = useReleaseTargetData
        if clearOrderCache is not None:
            taskparameters['clearOrderCache'] = clearOrderCache
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotname=robotname, robotspeed=robotspeed, robotaccelmult=robotaccelmult, timeout=timeout)

    def IsRobotOccludingBody(self, bodyname, cameraname, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Any, Any, float, Optional[types.IsRobotOccludingBodyParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[types.IsRobotOccludingBodyReturns]
        """
        returns if the robot is occluding body in the view of the specified camera

        Args:
            bodyname: Name of the object
            cameraname: Name of the camera
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            The occlusion state in a json dictionary, e.g. {'occluded': 0}
        """
        taskparameters = {
            'command': 'IsRobotOccludingBody',
            'bodyname': bodyname,
            'cameraname': cameraname,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetPickedPositions(self, unit='m', timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, float, Optional[types.GetPickedPositionsParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        returns the poses and the timestamps of the picked objects

        Args:
            unit: The unit of the given values. (Default: 'm')
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            The positions and the timestamps of the picked objects in a json dictionary, info of each object has the format of quaternion (w,x,y,z) followed by x,y,z translation (in mm) followed by timestamp in milisecond e.g. {'positions': [[1,0,0,0,100,200,300,1389774818.8366449],[1,0,0,0,200,200,300,1389774828.8366449]]}
        """
        taskparameters = {
            'command': 'GetPickedPositions',
            'unit': unit,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GetPickAndPlaceLog(self, timeout=10, startindex=None, num=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[int], Optional[int], Optional[types.GetPickAndPlaceLogParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Gets the recent pick-and-place log executed on the binpicking server. The internal server keeps the log around until the next Pick-and-place command is executed.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            startindex: Start of the trajectory to get. If negative, will start counting from the end. For example, -1 is the last element, -2 is the second to last element. (Default: None)
            num: Number of trajectories from startindex to return. If 0 will return all the trajectories starting from startindex (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)

        Returns:
            A dictionary with keys, for example:
            {
                total: 10
                messages: [{
                    "message":"message1",
                    "type":"",
                    "level":0,
                    "data": {
                        "jointvalues":[0,0,0,0,0,0]
                    }
                },
                ...
                ]
            }
        """
        taskparameters = {
            'command': 'GetPickAndPlaceLog',
        }  # type: dict[str, Any]
        if startindex is not None:
            taskparameters['startindex'] = startindex
        if num is not None:
            taskparameters['num'] = num
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def MoveRobotOutOfCameraOcclusion(self, regionname=None, robotspeed=None, toolname=None, timeout=10, cameranames=None, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, Optional[float], Optional[str], float, Optional[list[str]], Optional[types.MoveRobotOutOfCameraOcclusionParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Moves the robot out of camera occlusion and deletes targets if it was in occlusion.

        Args:
            regionname: Name of the region of the objects. (Default: None)
            robotspeed: Value in (0,1] defining the percentage of speed the robot should move at. (Default: None)
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            cameranames: The names of the cameras to avoid occlusions with the robot (Default: None)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        if regionname is None:
            regionname = self.regionname
        taskparameters = {
            'command': 'MoveRobotOutOfCameraOcclusion',
            'containername': regionname,
        }  # type: dict[str, Any]
        if cameranames is not None:
            taskparameters['cameranames'] = cameranames
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotspeed=robotspeed, toolname=toolname, timeout=timeout)

    def PausePickPlace(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, Optional[types.PausePickPlaceParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'PausePickPlace',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ResumePickPlace(self, timeout=10, dynamicEnvironmentState=None, debuglevel=None, numTargetsToPlace=None, enablerobotbridge=None, **kwargs):
        # type: (float, Optional[types.ResumePickPlaceParametersDynamicEnvironmentState], Optional[int], Optional[int], Optional[bool], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            numTargetsToPlace: The number of targets that are left to place (Default: None)
            enablerobotbridge: If True, enables robot bridge (Default: None)
        """
        taskparameters = {
            'command': 'ResumePickPlace',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if numTargetsToPlace is not None:
            taskparameters['numTargetsToPlace'] = numTargetsToPlace
        if enablerobotbridge is not None:
            taskparameters['enablerobotbridge'] = enablerobotbridge
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SendStateTrigger(self, stateTrigger, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, float, bool, Optional[types.SendStateTriggerParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            stateTrigger: a string that represents a unique trigger
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SendStateTrigger',
            'stateTrigger': stateTrigger,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetBinpickingState(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.GetBinpickingStateParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetBinpickingState',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetStopPickPlaceAfterExecutionCycle(self, finishStatus=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (Optional[str], float, Optional[types.SetStopPickPlaceAfterExecutionCycleParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Sets the cycle for stopping after the current pick cycle finishes.
        
        If robot has not grabbed a part yet, then will stop the robot immediately.
        On proper finish of the pick cycle, robot should go back to the finish position.

        Args:
            finishStatus: Optional finish code to end the cycle with (if it doesn't end with something else beforehand). (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetStopPickPlaceAfterExecutionCycle',
        }  # type: dict[str, Any]
        if finishStatus is not None:
            taskparameters['finishStatus'] = finishStatus
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def PutPartsBack(self, trajectory, numparts, toolname=None, grippervalues=None, timeout=100, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, int, str, Optional[list[Any]], float, Optional[types.PutPartsBackParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Runs saved planningresult trajectories.

        Args:
            trajectory:
            numparts:
            toolname: Name of the manipulator. Defaults to currently selected tool (Default: None)
            grippervalues: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 100)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'PutPartsBack',
            'trajectory': trajectory,
            'numparts': numparts,
            'toolname': toolname,
        }  # type: dict[str, Any]
        if grippervalues is not None:
            taskparameters['grippervalues'] = grippervalues
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def GenerateGraspModelFromIkParams(self, graspsetname, targeturi, toolname, robotname=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, str, Optional[str], Optional[str], float, Optional[types.GenerateGraspModelFromIkParamsParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Generates grasp model IK for given setup.

        Args:
            graspsetname: Name of the grasp set to use
            targeturi: uri of target scene, e.g. '4902201402644.mujin.dae'
            toolname: Name of the manipulator. Defaults to currently selected tool
            robotname: Name of the robot (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GenerateGraspModelFromIkParams',
            'graspsetname': graspsetname,
            'targeturi': targeturi,
            'toolname': toolname,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, robotname=robotname, toolname=toolname, timeout=timeout)

    def CheckGraspModelIk(self, graspsetname, targeturi, toolname, ikparamnames=None, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, Any, Optional[str], list[str], float, Optional[types.CheckGraspModelIkParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Checks if grasp model is generated for given setup.

        Args:
            graspsetname: Name of the grasp set to use
            targeturi: str. uri of target scene like 'mujin:4902201402644.mujin.dae'
            toolname: Name of the manipulator. Defaults to currently selected tool
            ikparamnames: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'CheckGraspModelIk',
            'graspsetname': graspsetname,
            'targeturi': targeturi,
            'toolname': toolname,
            'ikparamnames': ikparamnames,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def SetCurrentLayoutDataFromPLC(self, containername, containerLayoutSize, destObstacleName, ioVariableName, timeout=10, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (str, Any, str, str, float, Optional[types.SetCurrentLayoutDataFromPLCParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Sets current layout from PLC.

        Args:
            containername:
            containerLayoutSize:
            destObstacleName:
            ioVariableName:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetCurrentLayoutDataFromPLC',
            'containername': containername,
            'containerLayoutSize': containerLayoutSize,
            'destObstacleName': destObstacleName,
            'ioVariableName': ioVariableName,
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout)

    def ClearVisualization(self, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.ClearVisualizationParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Clears visualization.

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'ClearVisualization',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def GetPlanStatistics(self, timeout=1, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (float, bool, Optional[types.GetPlanStatisticsParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Gets plan and execute statistics of the last pick and place

        Args:
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 1)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'GetPlanStatistics',
        }  # type: dict[str, Any]
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

    def SetCurrentLayoutDataSendOnObjectUpdateData(self, doUpdate, containername=None, containerLayoutSize=None, ioVariableName=None, fireandforget=True, dynamicEnvironmentState=None, debuglevel=None, **kwargs):
        # type: (bool, Optional[str], Optional[Any], Optional[str], bool, Optional[types.SetCurrentLayoutDataSendOnObjectUpdateDataParametersDynamicEnvironmentState], Optional[int], Optional[Any]) -> Optional[Any]
        """
        Sets currentLayoutDataSendOnObjectUpdateData structure

        Args:
            doUpdate: If True then currentLayoutData will be send on every ObjectUpdate, else currentLayoutDataSendOnObjectUpdate structure is reset
            containername: (Default: None)
            containerLayoutSize: (Default: None)
            ioVariableName: (Default: None)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: True)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
        """
        taskparameters = {
            'command': 'SetCurrentLayoutDataSendOnObjectUpdateData',
            'doUpdate': doUpdate,
        }  # type: dict[str, Any]
        if containername is not None:
            taskparameters['containername'] = containername
        if containerLayoutSize is not None:
            taskparameters['containerLayoutSize'] = containerLayoutSize
        if ioVariableName is not None:
            taskparameters['ioVariableName'] = ioVariableName
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        taskparameters.update(kwargs)
        return self.ExecuteCommand(taskparameters, fireandforget=fireandforget)

    def ManuallyPlacePackItem(self, packFormationComputationResult=None, inputPartIndex=None, placeLocationNames=None, placedTargetPrefix=None, dynamicGoalsGeneratorParameters=None, orderNumber=None, numLeftToPick=None, timeout=10, fireandforget=False, dynamicEnvironmentState=None, debuglevel=None, unit='mm', sizeRoundUp=None, sizePrecisionXYZ=None, randomBoxOrigin=None):
        # type: (types.PackFormation, int, list[str], Optional[str], Optional[types.ManuallyPlacePackItemParametersDynamicGoalsGeneratorParameters], Optional[int], Optional[int], float, bool, Optional[types.ManuallyPlacePackItemParametersDynamicEnvironmentState], Optional[int], str, Optional[bool], Optional[tuple[float, float, float]], Optional[tuple[float, float, float]]) -> Optional[Any]
        """
        Places an item according to the pack formation assuming the item is placed manually and updates robotbridge state

        Args:
            packFormationComputationResult: A pack formation computed by Mujin. (Default: None)
            inputPartIndex: (Default: None)
            placeLocationNames: (Default: None)
            placedTargetPrefix: (Default: None)
            dynamicGoalsGeneratorParameters: If 'useDynamicGoals' is True, then will be dynamically generating goals based how to call on the packing algorithms. Internally, the packing algorithms parameters are managed by packFormation profiles.
             (Default: None)
            orderNumber: (Default: None)
            numLeftToPick: (Default: None)
            timeout: Time in seconds after which the command is assumed to have failed. (Default: 10)
            fireandforget: If True, does not wait for the command to finish and returns immediately. The command remains queued on the server. (Default: False)
            dynamicEnvironmentState: Dynamic environment state that allows the user to set/create objects in a particular state dynamically. (Default: None)
            debuglevel: Sets the debug level for the planning logs. For development. 3=INFO, 4=DEBUG, 5=VERBOSE. (Default: None)
            unit: The unit of the given values. (Default: 'mm')
            sizeRoundUp: (Default: None)
            sizePrecisionXYZ: mm (x,y,z) for rounding up incoming boxes from the detector. This allows previous grasping models to be cached and re-used since the sizes will be multiples of the current precision. (Default: None)
            randomBoxOrigin: Specifies where to place the origin of the incoming box detections. By default, this is [0,0,1], which means the origin will be at the center of the +Z (top) face. (Default: None)
        """
        taskparameters = {
            'command': 'ManuallyPlacePackItem',
            'packFormationComputationResult': packFormationComputationResult,
            'inputPartIndex': inputPartIndex,
            'placeLocationNames': placeLocationNames,
            'unit': unit,
        }  # type: dict[str, Any]
        if placedTargetPrefix:
            taskparameters['placedTargetPrefix'] = placedTargetPrefix
        if dynamicGoalsGeneratorParameters is not None:
            taskparameters['dynamicGoalsGeneratorParameters'] = dynamicGoalsGeneratorParameters
        if orderNumber is not None:
            taskparameters['orderNumber'] = orderNumber
        if numLeftToPick is not None:
            taskparameters['numLeftToPick'] = numLeftToPick
        if dynamicEnvironmentState is not None:
            taskparameters['dynamicEnvironmentState'] = dynamicEnvironmentState
        if debuglevel is not None:
            taskparameters['debuglevel'] = debuglevel
        if sizeRoundUp is not None:
            taskparameters['sizeRoundUp'] = sizeRoundUp
        if sizePrecisionXYZ is not None:
            taskparameters['sizePrecisionXYZ'] = sizePrecisionXYZ
        if randomBoxOrigin is not None:
            taskparameters['randomBoxOrigin'] = randomBoxOrigin
        return self.ExecuteCommand(taskparameters, timeout=timeout, fireandforget=fireandforget)

