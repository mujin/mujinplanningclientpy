# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
# Copyright (C) 2012-2023 Mujin, Inc.
# AUTO GENERATED FILE! DO NOT EDIT!

<%
def _SortedParams(params):
    # type: list[(str, dict[str, Any])] -> list[(str, dict[str, Any])]
    KWARG_PARAMS = ['kwargs', 'ignoredArgs']
    paramSortKey = lambda paramItem: paramItem[1].get('paramOrderingIndex', 998) if paramItem[0] not in KWARG_PARAMS else 999
    return sorted(params, key=paramSortKey)


def _ArgsForPayload(args):
    for name, data in args.items():
        if not any(map(lambda removalReason: data.get(removalReason, False),
            ['omit', 'deprecated', 'x-getFromPrepareCommandMethod', 'x-doNotAddToPayload'])):
            # We still need the name since it's used to as the key in the command.
            yield name, data


# TODO(heman.gandhi): Consider putting this in the client generators utilities.
def _TidyRawCode(code, indentationLevel, removeBlank=True, indentationString = ' '*4):
    """Indents multi-line strings to match the indentation they were interpolated at.

    As Mako interpolates the string, the first line is properly indented, but the subsequent ones
    lose initial indentation level. This code fixes the interpolation by adding the initial indent
    (specified as `indentationLevel`) to all but the first line (even if the first line is empty).

    Args:
        code (str): the multi-line string.
        indentationLevel (int): the number of indents the first line was at.
        removeBlank (bool): whether empty lines should be included. If they are included, they are unaltered.
        indentationString (str): represents the string used to move one indentation level.

    Returns:
        str: the indented string.
    """
    if not code:
        return ''
    lines = []
    isFirstLine = True
    for line in code.splitlines():
        if not removeBlank or len(line) > 0:
            if isFirstLine or len(line) == 0:
                lines.append(line)
            else:
                lines.append(indentationLevel * indentationString + line)
        isFirstLine = False
    return '\n'.join(lines)


def _FormatParameterList(serviceData):
    # Note: using `map` and `filter` since Mako doesn't parse list comprehensions.
    sortedKeptParams = _SortedParams(filter(lambda item: not item[1].get('omit', False), serviceData['parameters'].items()))
    formattedParams = ['self'] + list(map(lambda item: FormatMethodParameter(*item), sortedKeptParams))
    if len(formattedParams) > 20:
        formattedParams[0] = ' ' * 8 + formattedParams[0]
        return '\n' + _TidyRawCode(',\n'.join(formattedParams), 2) + '\n' + ' ' * 4
    return ', '.join(formattedParams)


def _AssignmentRightSide(paramName, paramData):
    rightSideParamName = paramData.get('customParameterName', paramName)
    if 'forceCastList' in paramData:
        return "[" + paramData['forceCastList'] + "(f) for f in " + rightSideParamName + "]"
    if 'forceCast' in paramData:
        return paramData['forceCast'] + '(' + rightSideParamName + ')'
    return rightSideParamName
%>
from . import ${parentClassFile}

import logging
log = logging.getLogger(__name__)

class ${clientTaskName}PlanningClient(${parentClassFile}.${parentClassName}):
    """Mujin planning client for the ${clientTaskName} task"""

    % if extraClassAttributes:
    ${_TidyRawCode(extraClassAttributes, 1, removeBlank=False)}

    % endif
    _deprecated = None # used to mark arguments as deprecated (set argument default value to this)

    def __init__(
        self, ${extraConstructorArgs} taskzmqport=11000, taskheartbeatport=11001, taskheartbeattimeout=7.0, tasktype='${tasktype}', scenepk='', ctx=None, slaverequestid=None, controllerip='', controllerurl='', controllerusername='', controllerpassword='', callerid='', **ignoredArgs
    ):
        """Connects to the Mujin controller, initializes ${clientTaskName} task and sets up parameters

        Args:
            ${_TidyRawCode(extraConstructorArgsDocstringLines, 3, removeBlank=False)}
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
        ${_TidyRawCode(extraConstructorContent, 2, removeBlank=False)}
        super(${clientTaskName}PlanningClient, self).__init__(
            % if extraSuperConstructorArgs:
            ${_TidyRawCode(extraSuperConstructorArgs, 3, removeBlank=False)}
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
    ${_TidyRawCode(extraClientStaticFunctions, 1, removeBlank=False)}
    %endif

    #
    # commands (Generated from the spec)
    #

% for serviceName, serviceData in spec['services'].items():
<%
    serviceData['parameters'] = serviceData['parameters']['taskparams']['properties']['taskparameters']['properties']
%>
    def ${serviceName}(${_FormatParameterList(serviceData)}):
        ${_TidyRawCode(FormatDocstring(serviceData, _SortedParams), 2, removeBlank=False)}
        % if serviceData.get('x-methodStartSetup'):
        ${_TidyRawCode(serviceData['x-methodStartSetup'], 2)}
        % endif
        command = self._PrepareCommand('${serviceData.get('serversideCommandName', serviceName)}'\
${", " if any(data.get('x-getFromPrepareCommandMethod', False) for data in serviceData['parameters'].values()) else ""}\
${', '.join(name + '=' + name for name, data in serviceData['parameters'].items() if data.get('x-getFromPrepareCommandMethod', False))})
    ## The for-loop omits args in this way because it's difficult to have a "continue" statement without also having an empty line
    % for paramName, paramData in _ArgsForPayload(serviceData['parameters']):
        % if 'x-specialCase' in paramData and 'content' in paramData['x-specialCase'] and not paramData['x-specialCase'].get('omitRegularAssignment', False):
        ${_TidyRawCode(paramData['x-specialCase']['content'], 2)}
        % endif
        % if 'x-specialCase' in paramData and paramData['x-specialCase'].get('omitRegularAssignment', False):
            %if 'content' in paramData['x-specialCase']:
        ${_TidyRawCode(paramData['x-specialCase']['content'], 2)}
            % else:
            % endif
        % elif paramName == 'kwargs':
        command.update(kwargs)
        % elif paramData.get('isRequired') or paramData.get('default') not in [None, False, '']:
        command['${paramData.get('mapsTo', paramName)}'] = ${_AssignmentRightSide(paramName, paramData)}
            % if paramData.get('mapsToParamWithCustomAssignment'):
        command['${paramData['mapsToParamWithCustomAssignment']['param']}'] = ${paramData['mapsToParamWithCustomAssignment']['assignment']}
            % endif
        % else:
        if ${paramData.get('customParameterName', paramName)} ${'!=' if paramData.get('default') is not None else 'is not'} ${repr(paramData.get('default'))}:
            command['${paramData.get('mapsTo', paramName)}'] = ${_AssignmentRightSide(paramName, paramData)}
            % if paramData.get('mapsToParamWithCustomAssignment'):
            command['${paramData['mapsToParamWithCustomAssignment']['param']}'] = ${paramData['mapsToParamWithCustomAssignment']['assignment']}
            % endif
            % if 'fillFromClassMember' in paramData:
        elif self.${paramData['fillFromClassMember']} ${'!=' if paramData.get('default') is not None else 'is not'} ${repr(paramData.get('default'))}:
            command['${paramData.get('mapsTo', paramName)}'] = self.${paramData['fillFromClassMember']}
            % endif
        % endif
    % endfor
    % if serviceData.get('x-modifiedReturnStatement'):
        ${_TidyRawCode(serviceData['x-modifiedReturnStatement'], 2)}
    % endif
    % if not serviceData.get('x-omitRegularReturnStatement', False):
        ${"return " if serviceData.get('returns') is not None else ''}\
self.ExecuteCommand\
(command, ${
    ', '.join(
        data.get('customParameterName', arg) + '=' + data.get('customParameterName', arg)
        for arg, data in serviceData['parameters'].items()
        if data.get('x-doNotAddToPayload', False)
    )
})\
${
    ".get('" + serviceData['returns']['returnPayloadField'] + "', None)" \
    if 'returnPayloadField' in serviceData.get('returns', {}) else \
    ''
}
    % endif

% endfor
% if extraClientContent:

% for line in extraClientContent.splitlines():
% if line.strip():
    ${line}
% else:

% endif
% endfor
% endif
