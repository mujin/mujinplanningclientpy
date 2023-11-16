# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

# system imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from typing import Any, Dict, List, Optional, Tuple, Union # noqa: F401 # used in type check

# mujin imports
% if templateArgs.get('extraImports') is not None:
${templateArgs['extraImports']}
% endif
from . import zmq
from . import ${templateArgs['parentClassFile']}

# logging
import logging
log = logging.getLogger(__name__)


class ${templateArgs['clientTaskName']}PlanningClient(${templateArgs['parentClassFile']}.${templateArgs['parentClassName']}):
    """Mujin planning client for the ${templateArgs['clientTaskName']} task"""

    % if templateArgs.get('extraClassAttributes') is not None:
    ${Indent(templateArgs['extraClassAttributes'], indentationLevel=1)}

    % endif
    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self,
        ${Indent(templateArgs['extraConstructorArgs'], indentationLevel=2)}
        taskzmqport=11000,
        taskheartbeatport=11001,
        taskheartbeattimeout=7.0,
        tasktype='${templateArgs['tasktype']}',
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
        # type: (${templateArgs['extraConstructorArgsTypeAnnotation']}, int, int, float, str, Optional[zmq.Context], Optional[str], str, str, str, str, str, str, Any) -> None
        """Connects to the Mujin controller, initializes ${templateArgs['clientTaskName']} task and sets up parameters

        Args:
            ${Indent(templateArgs['extraConstructorArgsDocstringLines'], indentationLevel=3)}
            taskzmqport (int, optional): Port of the task's ZMQ server, e.g. 7110. (Default: 11000)
            taskheartbeatport (int, optional): Port of the task's ZMQ server's heartbeat publisher, e.g. 7111. (Default: 11001)
            taskheartbeattimeout (float, optional): Seconds until reinitializing task's ZMQ server if no heartbeat is received, e.g. 7
            tasktype (str, optional): Type of the task, e.g. 'binpicking', 'handeyecalibration', 'itlrealtimeplanning3'. Default: ${templateArgs['tasktype']}
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
        ${Indent(templateArgs['extraConstructorContent'], indentationLevel=2)}
        super(${templateArgs['clientTaskName']}PlanningClient, self).__init__(
            % if templateArgs.get('extraSuperConstructorArgs') is not None:
            ${Indent(templateArgs['extraSuperConstructorArgs'], indentationLevel=3)}
            % endif
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

    % if templateArgs.get('extraClientStaticFunctions') is not None:
    ${Indent(templateArgs['extraClientStaticFunctions'], indentationLevel=1)}
    %endif

    #
    # Commands (generated from the spec)
    #

% for serviceName, serviceData in services.items():
<%
    clientAvailableParams = []
    for rootParameter in serviceData['parameters']:
        if rootParameter['name'] == 'taskparams':
            clientAvailableParams = [
                {
                    'name': paramName,
                    'schema': paramData
                }
                for paramName, paramData in rootParameter['schema']['properties']['taskparameters']['properties'].items()
            ]
            break
    serviceData['parameters'] = clientAvailableParams
%>
    <%include file="/serviceTemplate.py.mako" args="serviceName=serviceName,serviceData=serviceData,executeCommandMethod='ExecuteCommand',usePrepareCommand=False,commandVariable='taskparameters'" />\
% endfor
% if templateArgs['extraClientContent']:

% for line in templateArgs['extraClientContent'].splitlines():
% if line.strip():
    ${line}
% else:

% endif
% endfor
% endif
