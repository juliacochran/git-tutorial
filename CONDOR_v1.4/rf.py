# CONDOR
# Integrated Product Lifecylce Engineering Labratory

import math
from vehicle import Vehicle

def pvar(locals_, vars_):
    """This is a print command used to print values in debug mode"""
    s = ['%s: %d' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

class SizedVehicle:

    def __init__(self, vconfig, mconfig, airfoildata_mainRotor, Master):
        self.vconfig = vconfig
        self.mconfig = mconfig
        self.vconfig['Sizing Results']['SizedGrossWeight'] = float('nan')

        self.airfoildata_mainRotor = airfoildata_mainRotor
        self.debug = Master['RF Options']['RF_debug']
        self.writeOutput = Master['RF Options']['RF_writeOutput']
        self.Master = Master        
        
    def sizeMission(self):
        """This is the new sizing routine.  It uses bracketing to narrow the tolerances
        of the GW solution until it is found to be within the specified accuracy.  Should
        be totally stable, and doesn't take too many iterations even with wide limits."""
        v = self.vconfig
        m = self.mconfig
        steps = 0
        GWmin = v['Simulation']['GWMin']
        GWmax = v['Simulation']['GWmax']
        viableCandidate = False
        goodAbove = False
        GW = (GWmax + GWmin) / 2
        # http://www.youtube.com/watch?v=Xs_OacEq2Sk
        choppah = Vehicle(v, m, GW, self.airfoildata_mainRotor,self.Master)
        choppah.flyMission()
        while steps<choppah.vconfig['Simulation']['MaxSteps'] and (GWmax-GWmin)>choppah.vconfig['Simulation']['GWTolerance']:
            # Depending on whether we're oversized or undersized for the mission, adjust our GW limits accordingly
            if choppah.vconfig['Sizing Results']['CouldTrim']:
                if choppah.misSize > 0: # we can trim and we're too big
                    GWmax = GW
                    goodAbove = choppah
                    viableCandidate = True
                else: # we can trim and we're too small
                    GWmin = GW
            else: # if we can't trim the current candidate
                if goodAbove: # we can't trim but we could when we were heavier
                    GWmin = GW
                else: # we can't trim and we never could
                    GWmax = GW
            GW = (GWmax - GWmin) / 2 + GWmin
            choppah = Vehicle(v, m, GW, self.airfoildata_mainRotor,self.Master)
            choppah.flyMission()
            steps += 1

            if self.debug:
                couldTrim = choppah.vconfig['Sizing Results']['CouldTrim']
                couldMission = choppah.misSize > 0
                ms = 99999999999999 if math.isnan(choppah.misSize) else choppah.misSize
                gA = goodAbove is not False
                gAW = goodAbove.vconfig['Sizing Results']['GrossWeight'] if goodAbove else -999999999
                pvar(locals(), ('steps', 'GWmax', 'GWmin', 'couldTrim', 'couldMission'))
        stopReason = ''
        goodRun = False
#        if not (choppah.vconfig['Sizing Results']['CouldTrim'] and choppah.misSize>0):
#            choppah = goodAbove
        if choppah:
            if not choppah.vconfig['Sizing Results']['CouldTrim']:
                stopReason = 'Cound not trim at all conditions at any mission-capable weight'
            elif choppah.vconfig['Weights']['MaxAvailableFuelWeight'] < 0:
                stopReason = 'Negative calculated max fuel weight'
            elif steps >= choppah.vconfig['Simulation']['MaxSteps']:
                stopReason = 'MaxSteps reached before convergance.  Stopped with bounds: %f  to  %f' % (GWmin, GWmax)
            elif (GWmax-GWmin <= choppah.vconfig['Simulation']['GWTolerance']):
                stopReason = 'Converged to within specified tolerances'
                goodRun = True
            else:
                stopReason = 'Stopped with some other reason'
            choppah.vconfig['Sizing Results']['StopReason'] = stopReason
            choppah.vconfig['Sizing Results']['GoodRun'] = goodRun
            if goodRun:
                choppah.vconfig['Sizing Results']['SizedWeightFound'] = True
                choppah.vconfig['Sizing Results']['SizedGrossWeight'] = GW
            else:
                choppah.vconfig['Sizing Results']['SizedWeightFound'] = False
                choppah.vconfig['Sizing Results']['SizedGrossWeight'] = float('nan')
            if self.debug: print('SizedWeightFound: %s     %s' % (goodRun, stopReason))
            if self.writeOutput: choppah.write()
        return choppah

