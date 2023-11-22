# -*- coding: utf-8 -*-
# Copyright (C) 2012-2014 MUJIN Inc
from distutils.core import setup
try:
    from mujincommon.setuptools import Distribution
except (ImportError, SyntaxError):
    from distutils.dist import Distribution

version = {}
exec(open('python/mujinplanningclient/version.py').read(), version)

setup(
    distclass=Distribution,
    name='mujinplanningclient',
    version=version['__version__'],
    packages=['mujinplanningclient', 'mujinplanningapi'],
    package_dir={
        'mujinplanningclient': 'python/mujinplanningclient',
        'mujinplanningapi': 'python/mujinplanningapi',
    },
    data_files=[
        # using scripts= will cause the first line of the script being modified for python2 or python3
        # put the scripts in data_files will copy them as-is
        ('bin', [
            'bin/mujin_planningclientpy_runshell.py',
        ]),
    ],
    locale_dir='locale',
    license='Apache License, Version 2.0',
    long_description=open('README.md').read(),
    # flake8 compliance configuration
    enable_flake8=True,  # Enable checks
    fail_on_flake=True,  # Fail builds when checks fail
    install_requires=[],
    package_data={'mujinplanningapi': ['templates/*.mako']},
    api_spec = [
        'mujinplanningapi.spec_realtimerobot.realtimeRobotSpec',
        'mujinplanningapi.spec_realtimeitl3.realtimeITL3Spec',
        'mujinplanningapi.spec_handeyecalibration.calibrationSpec',
        'mujinplanningapi.spec_binpicking.binpickingSpec',
    ],
    generate_packages=[
        (
            '_generatedclient1',
            'mujinclientgenerators.generateClient.GenerateClient',
            {
                'specDictName': 'mujinplanningapi.spec_handeyecalibration.calibrationSpec',
                'generatorSettingsDictName': 'mujinplanningapi.generator_settings_python_handeyecalibration.generatorSettings',
                'output': './python/mujinplanningclient/handeyecalibrationplanningclient.py',
                'language': 'python',
            },
        ),
        (
            '_generatedclient2',
            'mujinclientgenerators.generateClient.GenerateClient',
            {
                'specDictName': 'mujinplanningapi.spec_binpicking.binpickingSpec',
                'generatorSettingsDictName': 'mujinplanningapi.generator_settings_python_binpicking.generatorSettings',
                'output': './python/mujinplanningclient/binpickingplanningclient.py',
                'language': 'python',
            },
        ),
        (
            '_generatedclient3',
            'mujinclientgenerators.generateClient.GenerateClient',
            {
                'specDictName': 'mujinplanningapi.spec_realtimerobot.realtimeRobotSpec',
                'generatorSettingsDictName': 'mujinplanningapi.generator_settings_python_realtimerobot.generatorSettings',
                'output': './python/mujinplanningclient/realtimerobotplanningclient.py',
                'language': 'python',
            },
        ),
        (
            '_generatedclient4',
            'mujinclientgenerators.generateClient.GenerateClient',
            {
                'specDictName': 'mujinplanningapi.spec_realtimeitl3.realtimeITL3Spec',
                'generatorSettingsDictName': 'mujinplanningapi.generator_settings_python_realtimeitl3.generatorSettings',
                'output': './python/mujinplanningclient/realtimeitl3planningclient.py',
                'language': 'python',
            },
        ),
    ],
)
