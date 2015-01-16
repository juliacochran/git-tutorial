# -*- coding: utf-8 -*-
"""
Created on Fri Oct 24 13:49:13 2014

@author: bengland3
"""

""" This code needs to be where the independent run file is for BEMT"""
import math
import numpy as np

def run_BEMT(Master):
    debug = Master['BEMT Options']['BEMT_debug']
    plot = Master['BEMT Options']['BEMT_plot']
    animate =Master['BEMT Options']['BEMT_animate']
    runTests = False
    from time import clock
    from configobj import ConfigObj
    from validate import Validator
    from BEMT import Blade, Rotor
    import matplotlib.pyplot as plt
    startTime = clock()
    if runTests:
        fails = []
        for GW in [5000., 50000., 80000.]:
            for V in [0., 300.]:
                for horizM in [1., 10.]:
                    for vertM in [1.,  10.]:
                        for balance in [.5, 1.]:
                            s = 'GW: %d     V: %d     horizM: %d     vertM: %d     balance: %f' % (GW, V, horizM, vertM, balance)
                            print s
                            v = ConfigObj(Master['Veh Mission Config']['Aircraft_Config'], configspec='Config/Configspec/vehicle.configspec')
                            m = ConfigObj(Master['Veh Mission Config']['Mission_Config'], configspec='Config/Configspec/mission.configspec')
                            vvdt = Validator()
                            v.validate(vvdt)
                            mvdt = Validator()
                            m.validate(mvdt)
                            rho = 0.0024
                            f = 0.25 * GW**.5 * (1-v['Body']['DragTechImprovementFactor'])
                            Vtip = v['Main Rotor']['TipSpeed'] # ft/s
                            R = math.sqrt(GW / (math.pi * v['Main Rotor']['DiskLoading'] * v['Main Rotor']['NumRotors']))
                            v['Main Rotor']['Radius'] = R
                            v['Main Rotor']['DiskArea'] = math.pi * v['Main Rotor']['Radius']**2 * v['Main Rotor']['NumRotors']
                            v['Main Rotor']['AverageChord'] = v['Main Rotor']['DiskArea']*v['Main Rotor']['Solidity'] / (v['Main Rotor']['Radius']*(1-v['Main Rotor']['RootCutout'])*v['Main Rotor']['NumBlades'])
                            omega = Vtip / R # rad/s
                            c81File='Config/C81/%s'%v['Main Rotor']['AirfoilFile']
                            airfoildata = np.genfromtxt(c81File, skip_header=0, skip_footer=0) # read in the airfoil file
                            averageChord=v['Main Rotor']['AverageChord']/v['Main Rotor']['Radius']
                            blade = Blade(airfoildata,averageChord, Master,\
                                          skip_header=0, skip_footer=0, \
                                          taperRatio=v['Main Rotor']['TaperRatio'], tipTwist=v['Main Rotor']['TipTwist'], \
                                          rootCutout=v['Main Rotor']['RootCutout'], segments=v['Simulation']['numBladeElementSegments'], \
                                          dragDivergenceMachNumber=v['Main Rotor']['DragDivergenceMachNumber'])
                            psiSegments=v['Simulation']['numBladeElementSegments']
                            Vtip=v['Main Rotor']['TipSpeed']
                            radius=v['Main Rotor']['Radius']
                            numblades=v['Main Rotor']['NumBlades']
                            rotor = Rotor(blade, psiSegments, Vtip, radius, numblades, Master)
                            bladeArea = np.sum(blade.chord * rotor.radius * blade.dr * rotor.radius * rotor.numblades)
                            diskArea = math.pi * rotor.radius**2
                            solidity = bladeArea / diskArea
                            Fhorizontal = 1./2 * rho * V**2 * f / horizM
                            Fvertical = GW / vertM
                            temp = rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fz=Fvertical, maxSteps=v['Simulation']['MaxSteps'], advancingLiftBalance=balance) + Fhorizontal*V/550
                            if math.isnan(temp) or temp<0:
                                fails.append(s)
                                print('FAIL')
        print ''
        print ''
        print ''
        print 'FAILURES:'
        for fail in fails:
            print fail
    else:
        v = ConfigObj(Master['Veh Mission Config']['Aircraft_Config'], configspec='Config/Configspec/vehicle.configspec')
        m = ConfigObj(Master['Veh Mission Config']['Mission_Config'], configspec='Config/Configspec/mission.configspec')
        vvdt = Validator()
        v.validate(vvdt)
        mvdt = Validator()
        m.validate(mvdt)
        GW = v['Weights']['BaselineGrossWeight'] # 17613#
        V = 150.#270#
        V *= 1.687
        horizM = 1.
        vertM = 1.
        balance = .5
        s = 'GW: %d     V: %d     horizM: %d     vertM: %d     balance: %f' % (GW, V, horizM, vertM, balance)
        print s
        rho = density(v['Engine Scaling']['CruiseAltitude'],v['Engine Scaling']['DeltaTemp'])
        f = 0.25 * GW**.5 * (1-v['Body']['DragTechImprovementFactor'])
        Vtip = v['Main Rotor']['TipSpeed'] # ft/s
        R = math.sqrt(GW / (math.pi * v['Main Rotor']['DiskLoading'] * v['Main Rotor']['NumRotors']))
        v['Main Rotor']['Radius'] = R
        v['Main Rotor']['DiskArea'] = math.pi * v['Main Rotor']['Radius']**2 * v['Main Rotor']['NumRotors']
        v['Main Rotor']['AverageChord'] = 1. # v['Main Rotor']['DiskArea']*v['Main Rotor']['Solidity'] / (v['Main Rotor']['Radius']*(1-v['Main Rotor']['RootCutout'])*v['Main Rotor']['NumBlades'])
        omega = Vtip / R # rad/s
        c81File='Config/C81/%s'%v['Main Rotor']['AirfoilFile']
        airfoildata = np.genfromtxt(c81File, skip_header=0, skip_footer=0) # read in the airfoil file
        averageChord=v['Main Rotor']['AverageChord']/v['Main Rotor']['Radius']
        blade = Blade(airfoildata,averageChord, Master, skip_header=0, skip_footer=0,\
                      taperRatio=v['Main Rotor']['TaperRatio'], tipTwist=v['Main Rotor']['TipTwist'], \
                      rootCutout=v['Main Rotor']['RootCutout'], segments=v['Simulation']['numBladeElementSegments'], \
                      dragDivergenceMachNumber=v['Main Rotor']['DragDivergenceMachNumber'])
        psiSegments=v['Simulation']['numBladeElementSegments']
        Vtip=v['Main Rotor']['TipSpeed']
        radius=v['Main Rotor']['Radius']
        numblades=v['Main Rotor']['NumBlades']
        rotor = Rotor(blade, psiSegments, Vtip, radius, numblades, Master)
        bladeArea = np.sum(blade.chord * rotor.radius * blade.dr * rotor.radius * rotor.numblades)
        diskArea = math.pi * rotor.radius**2
        solidity = bladeArea / diskArea
        Fhorizontal = 1./2 * rho * V**2 * f / horizM
        Fvertical = GW / vertM
        print 'Total Power (HP):  ',rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], V=V, rho=rho, speedOfSound=1026., Fx=Fhorizontal, Fz=Fvertical, maxSteps=v['Simulation']['MaxSteps'], advancingLiftBalance=balance) + Fhorizontal*V/550


        if plot:
            plt.figure()
            plt.subplot(241)
            plt.plot(rotor.thrust_hist[5:])
            plt.title('thrust')

            plt.subplot(242)
            plt.plot(rotor.pitch_hist[5:])
            plt.title('pitch')

            plt.subplot(243)
            plt.plot(rotor.roll_hist[5:])
            plt.title('roll')

            plt.subplot(244)
            plt.plot(rotor.miscA_hist[5:])
            plt.title('rearLiftProportion')

            plt.subplot(245)
            plt.plot(rotor.theta_0_hist[5:])
            plt.title('theta_0')

            plt.subplot(246)
            plt.plot(rotor.t1c_hist[5:])
            plt.title('t1c')

            plt.subplot(247)
            plt.plot(rotor.t1s_hist[5:])
            plt.title('t1s')

            plt.subplot(248)
            plt.plot(rotor.miscB_hist[5:])
            plt.title('advancingLiftProportion')

            plt.show()
    stopTime = clock()

    elapsed = stopTime - startTime
    print('elapsed time: %d' % elapsed)

def pvar(locals_, vars_):
    '''Prints the variables and there name'''
    s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

def pdeg(locals_, vars_):
    s = ['%s: %.2f' % (var*180./math.pi, locals_[var]) for var in vars_]
    print '     '.join(s)
def density(altitude, DeltaTemp):
    ISA = np.genfromtxt("Config\Standard_Atmosphere\ISA_Standard_Atmosphere.txt", skip_header=2, skip_footer = 0)
    alt = ISA[:,0]
    temp = ISA[:,1]
    Pres = ISA[:,3]
    # ISA Standard Atmosphere Look up
    pressure = np.interp(altitude, alt, Pres) * 144.0 # Conversion to lb/ft^2
    temperature = np.interp(altitude,alt,temp) + 459.67 # Conversion to Rankin
    temperature += DeltaTemp
    R = 1716. # Gas costant Air imperial units (ft lb/slug Rankin)
    rho = pressure/(R*temperature)
    return rho #5e-13*altitude**2 - 7e-8*altitude + .0024 # ISA+15C?(slug/ft^3)