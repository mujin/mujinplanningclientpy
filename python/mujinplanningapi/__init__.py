# -*- coding: utf-8 -*-

import mujincommon.i18n
from mujincommon.dictutil import MergeDicts, MergeListsByKey

ugettext, ungettext = mujincommon.i18n.GetDomain('planningclient').GetTranslationFunctions()
_ = ugettext


def UpdateTaskparams(parameters, taskparamsDict):
    return MergeListsByKey(parameters, [taskparamsDict], lambda x: x['name'], lambda x, y: MergeDicts([x, y], deepcopy=True))
