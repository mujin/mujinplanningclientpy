# -*- coding: utf-8 -*-

import mujincommon.i18n
from mujincommon.dictutil import MergeDicts, MergeListsByKey

ugettext, ungettext = mujincommon.i18n.GetDomain('planningclient').GetTranslationFunctions()
_ = ugettext


def UpdateParameters(base, custom):
    return MergeListsByKey(base, custom, lambda x: x['name'], lambda x, y: MergeDicts([x, y]))
