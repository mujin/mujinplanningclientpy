# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

from . import ${parentClassFile}

import logging
log = logging.getLogger(__name__)

class ${clientTaskName}PlanningClient(${parentClassFile}.${parentClassName}):
    """Mujin planning client for the ${clientTaskName} task"""

    % if extraClassAttributes:
    ${FormatRawCode(extraClassAttributes, 1, removeBlank=False)}

    % endif
    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self, ${extraConstructorArgs} taskzmqport=11000, taskheartbeatport=11001, taskheartbeattimeout=7.0, tasktype='${tasktype}', scenepk='', ctx=None, slaverequestid=None, controllerip='', controllerurl='', controllerusername='', controllerpassword='', callerid='', **ignoredArgs
    ):
        """Connects to the Mujin controller, initializes ${clientTaskName} task and sets up parameters

        Args:
            ${FormatRawCode(extraConstructorArgsDocstringLines, 3, removeBlank=False)}
            taskzmqport (int, optional): Port of the task's ZMQ server. Default: 11000
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher. Default: 11001
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: ${tasktype}
            scenepk (str, optional): Primary key (pk) of the scene, e.g. irex_demo.mujin.dae
            ctx (zmq.Context, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            slaverequestid:
            controllerip (str): IP or hostname of the mujin controller, e.g. 172.17.0.2 or controller123
            controllerurl (str, optional): (Deprecated. Use controllerip instead) URL of the mujin controller, e.g. http://controller14.
            controllerusername (str): Username for the Mujin controller, e.g. testuser
            controllerpassword (str): Password for the Mujin controller
            callerid (str, optional): Caller identifier to send to server on every command
            ignoredArgs: Additional keyword args are not used, but allowed for easy initialization from a dictionary
        """
        ${FormatRawCode(extraConstructorContent, 2, removeBlank=False)}
        super(${clientTaskName}PlanningClient, self).__init__(
            % if extraSuperConstructorArgs:
            ${FormatRawCode(extraSuperConstructorArgs, 3, removeBlank=False)}
            % endif
            taskzmqport=taskzmqport,
            taskheartbeatport=taskheartbeatport,
            taskheartbeattimeout=taskheartbeattimeout,
            tasktype=tasktype,
            scenepk=scenepk,
            ctx=ctx,
            slaverequestid=slaverequestid,
            controllerip=controllerip,
            controllerurl=controllerurl,
            controllerusername=controllerusername,
            controllerpassword=controllerpassword,
            callerid=callerid
        )

    % if extraClientStaticFunctions:
    ${FormatRawCode(extraClientStaticFunctions, 1, removeBlank=False)}
    %endif

    #
    # commands (Generated from the spec)
    #

% for serviceName, serviceData in spec['services'].items():
<%
    serviceData['parameters'] = serviceData['parameters']['taskparams']['properties']['taskparameters']['properties']
%>
    <%include file="/serviceTemplate.py.mako" args="serviceName=serviceName,serviceData=serviceData,executeCommandMethod='ExecuteCommand'" />
% endfor
% if extraClientContent:

% for line in extraClientContent.splitlines():
% if line.strip():
    ${line}
% else:

% endif
% endfor
% endif
