# -*- coding: utf-8 -*-
"""
Created on Fri Oct 24 13:49:14 2014

@author: bengland3
"""

""" This code needs to be where the independent run file is for RF"""

from rf import SizedVehicle

def run_rf(Master):
    from time import clock
    from configobj import ConfigObj
    from validate import Validator
    import numpy as np
    v = ConfigObj(Master['Veh Mission Config']['Aircraft_Config'], configspec='Config/Configspec/vehicle.configspec')
    m = ConfigObj(Master['Veh Mission Config']['Mission_Config'], configspec='Config/Configspec/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    startTime = clock()
    c81File_mainRotor = 'Config/C81/%s' % v['Main Rotor']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    blah = SizedVehicle(v, m, airfoildata_mainRotor, Master)
    veh = blah.sizeMission()
    if veh: veh.write()
    stopTime = clock()
    elapsed = stopTime - startTime
    if Master['RF Options']['RF_debug']: print('Elapsed time: %f' % elapsed)

def pvar(locals_, vars_):
    """This is a print command used to print values in debug mode"""
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)
