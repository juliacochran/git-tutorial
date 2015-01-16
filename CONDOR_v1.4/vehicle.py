# CONDOR
# Integrated Product Lifecylce Engineering Labratory
import math
from configobj import ConfigObj
from BEMT import Blade, Rotor
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
#from time import clock

def pvar(locals_, vars_):
  s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
  print '     '.join(s)

class Vehicle:

  def __init__(self, vconfig, mconfig, GW, airfoildata_mainRotor, Master):
      self.vconfig = ConfigObj(vconfig)
      self.mconfig = ConfigObj(mconfig)
      self.GW = GW
      self.airfoildata_mainRotor = airfoildata_mainRotor
      
      self.debug = Master['Vehicle Options']['vehicle_debug']
      self.debugFine = Master['Vehicle Options']['vehicle_debugFine']
      self.Master = Master
      
      self.ISA = np.genfromtxt("Config\Standard_Atmosphere\ISA_Standard_Atmosphere.txt", skip_header=2, skip_footer = 0)
      self.alt = self.ISA[:,0]
      self.temp = self.ISA[:,1]
      self.Pres = self.ISA[:,3]
      self.setup()

  def setup(self):
      v = self.vconfig
      m = self.mconfig
      GW = self.GW
      self.misSize = float('nan')

      # Engine scaling
      v['Condition']['CruiseAltitude'] = v['Engine Scaling']['CruiseAltitude']

#      v['Performance']['IngressSpeed'] = m['Segment 2']['Speed'] # REMOVEME This needs to be the max continuous power speed (Set after power curve generated)
#      v['Performance']['MissionRange'] = m['Segment 2']['Distance'] # REMOVEME this needs to be the sum of all mission compoenets (Set after flyMission is executed)

      v['Performance']['MaxBladeLoadingSeen'] = 0.

      if self.Master['Code Selection']['rf_Run']:
        v['Body']['FlatPlateDrag'] = 0.15 * GW**.5 * (1-v['Body']['DragTechImprovementFactor']) #0.015 * GW**0.67 # flat plate drag area
      else:
        v['Body']['FlatPlateDrag'] = v['Body']['BaselineFlatPlate']
      
      v['Main Rotor']['Radius'] = math.sqrt(GW / (math.pi * v['Main Rotor']['DiskLoading'] * v['Main Rotor']['NumRotors']))
      v['Main Rotor']['Omega'] = v['Main Rotor']['TipSpeed'] / v['Main Rotor']['Radius']
      v['Main Rotor']['DiskArea'] = math.pi * v['Main Rotor']['Radius']**2 * v['Main Rotor']['NumRotors']
      v['Main Rotor']['AverageChord'] = v['Main Rotor']['DiskArea']*v['Main Rotor']['Solidity'] / (v['Main Rotor']['Radius']*(1-v['Main Rotor']['RootCutout'])*v['Main Rotor']['NumBlades'])

      

      v['Sizing Results']['GrossWeight'] = GW
      v['Sizing Results']['CouldTrim'] = True
      v['Sizing Results']['MisSize'] = float('nan')
      v['Sizing Results']['Nothing'] = 0.
      v['Performance']['Nothing'] = 0.
      
      self.blade = Blade(airfoildata=self.airfoildata_mainRotor, Master = self.Master,skip_header=0, skip_footer=0, averageChord=v['Main Rotor']['AverageChord']/v['Main Rotor']['Radius'], taperRatio=v['Main Rotor']['TaperRatio'], tipTwist=v['Main Rotor']['TipTwist'], rootCutout=v['Main Rotor']['RootCutout'], segments=v['Simulation']['numBladeElementSegments'], dragDivergenceMachNumber=v['Main Rotor']['DragDivergenceMachNumber'])
      self.rotor = Rotor(self.blade, psiSegments=v['Simulation']['numBladeElementSegments'], Vtip=v['Main Rotor']['TipSpeed'], radius=v['Main Rotor']['Radius'], numblades=v['Main Rotor']['NumBlades'],Master = self.Master)
      self.sizeAntiTorque()      
      if self.Master['Code Selection']['rf_Run']: # When using RF method scales engine
        self.scaleEngine()
      else: # When calculating performance of vehicle uses input values
        v['Powerplant']['MRP'] = v['Powerplant']['BaselineMRP']*v['Weights']['NumEngines']
        v['Powerplant']['MCP'] = v['Powerplant']['BaselineMCP']*v['Weights']['NumEngines']
        v['Powerplant']['SFC'] = v['Powerplant']['BaselineSFC']

      self.scaleWeights()
      self.findCost()
      self.setMission(m)
      



  def findCost(self):
      v = self.vconfig
      HarrisScullyPrice = 628.707 * (v['Main Rotor']['NumBlades']*v['Main Rotor']['NumRotors'])**0.2045 * v['Weights']['EmptyWeight']**0.4854 * v['Powerplant']['MRP']**0.5843
      v['Economics']['HarrisScullyPrice'] = HarrisScullyPrice

  def scaleWeights(self):
      v = self.vconfig
      w = v['Weights']  # shorthand

      # calculation of baseline weight breakdown
      baselineEmptyWeight = w['BaselineEmptyWeight']
      baselineGrossWeight = w['BaselineGrossWeight']
      baselineEngineWeight = w['NumEngines']*w['BaselineWeightPerEngine']
      baselineDriveSystemWeight = w['BaselineWeightPerEngine']*w['BaselineDriveSystemWeightScalingFactor']
      baselineStructureWeight = baselineEmptyWeight - baselineEngineWeight - baselineDriveSystemWeight

      # calculation of baseline empty weight fractions
      w['baselineEmptyWeightFraction'] = baselineEmptyWeight / baselineGrossWeight
      w['baselineEngineWeightFraction'] = baselineEngineWeight / baselineGrossWeight
      w['baselineDriveSystemWeightFraction'] = baselineDriveSystemWeight / baselineGrossWeight
      baselineStructureWeightFraction = baselineStructureWeight / baselineGrossWeight
      improvedStructureWeightFraction = baselineStructureWeightFraction * (1-w['StructureWeightTechImprovementFactor'])

      # weight scaling if RF method
      if self.Master['Code Selection']['rf_Run']:
        MRP = v['Powerplant']['MRP']
        scaledEngineWeight = w['NumEngines']*((0.1054*(MRP/w['NumEngines'])**2.+358*(MRP/w['NumEngines'])+2.757*10.**4.)/((MRP/w['NumEngines'])+1180))
      else:
        MRP = v['Powerplant']['MRP']
        scaledEngineWeight = w['NumEngines']*w['BaselineWeightPerEngine']
      
      try:
          scaledDriveSystemWeight = (525.*(self.GW/1000.)**1.14)/((self.GW/MRP)**0.763*v['Main Rotor']['DiskLoading']**0.381)
      except:
          scaledDriveSystemWeight = 999999999.
          print scaledDriveSystemWeight
          print MRP
      scaledStructureWeight = improvedStructureWeightFraction * self.GW
      
      if self.Master['Code Selection']['rf_Run']:
        v['Weights']['scaledEngineWeight'] = scaledEngineWeight * (1-v['Weights']['EngineWeightTechImprovementFactor'])
        v['Weights']['scaledDriveSystemWeight'] = scaledDriveSystemWeight * (1-v['Weights']['DriveSystemWeightTechImprovementFactor'])
        v['Weights']['scaledStructureWeight'] = scaledStructureWeight * (1-v['Weights']['StructureWeightTechImprovementFactor'])
        
        v['Weights']['EmptyWeightFraction'] = (scaledEngineWeight + scaledDriveSystemWeight + scaledStructureWeight) / self.GW - 0.11
        v['Weights']['EmptyWeight'] = v['Weights']['EmptyWeightFraction'] * self.GW
        v['Weights']['MaxAvailableFuelWeight'] = self.GW - v['Weights']['EmptyWeight'] - v['Weights']['UsefulLoad']
      else:
        v['Weights']['scaledEngineWeight'] = scaledEngineWeight
        v['Weights']['scaledDriveSystemWeight'] = scaledDriveSystemWeight
        v['Weights']['scaledStructureWeight'] = scaledStructureWeight
        
        # output
        v['Weights']['EmptyWeightFraction'] = w['baselineEmptyWeightFraction']
        v['Weights']['EmptyWeight'] = baselineEmptyWeight
        v['Weights']['MaxAvailableFuelWeight'] = self.GW - v['Weights']['EmptyWeight'] - v['Weights']['UsefulLoad']

  def setMission(self, mconfig):
      """This function sets the mission fleshing it out with some calculated values"""
      self.mconfig = mconfig
      m = self.mconfig
      v = self.vconfig
      segment = 1
      numsegs = 0
      prevLoad = 0
      maxLoad = 0
      while 'Segment %s' % segment in m:
          seg = 'Segment %s' % segment   
          m[seg]['Density']=self.density(m[seg]['Altitude'], m[seg]['DeltaTemp'])
          m[seg]['DensityRatio'] = m[seg]['Density'] / self.density(0,m[seg]['DeltaTemp'])
          load = m[seg]['CrewWeight'] + m[seg]['PayloadWeight'] + m[seg]['MiscWeight']
          maxLoad = max(maxLoad, load)
          m[seg]['DeltaLoad'] = load - prevLoad
          prevLoad = load
          if m[seg]['ClimbSegment']==True:
              if segment > 1:
                  m[seg]['StartAltitude'] = m['Segment %s' % str(segment-1)]['Altitude']
              else:
                  m[seg]['StartAltitude'] = m[seg]['Altitude']
                  
          m[seg]['95_powerAvail'] = self.powerAvailable(m[seg]['Altitude'])*.95

# Set In-Ground-Effect Thrust Multiplyer (assumes heightAboveGround/RotorRadius = .75) 
# The model used is Explained on Pgs 258-260 Leishman
          z_R = .75 # Potentially Change this to be an input ?????????????????
          if m[seg]['IGE']==True:
              IGEMult = 1/(1-(z_R**2.0 * 0.0625)) # 1/(1-(R/4z)**2)= T/T_inf
          else:
              IGEMult = 1.0 # No Ground Effect (Divide by 1)
          m[seg]['IGE Multiplier'] = IGEMult

          segment += 1
          numsegs += 1

      v['Sizing Results']['Payload'] = maxLoad
      v['Sizing Results']['TotalWeight'] = float('nan')
      v['Sizing Results']['FuelUsedLb'] = float('nan')
      m['MaxLoad'] = maxLoad
      m['NumSegments'] = numsegs

  def flyMission(self):
      m = self.mconfig
      v = self.vconfig
      v['Sizing Results']['MisSize'] = float('nan')
      if v['Weights']['EmptyWeightFraction']>1 or v['Weights']['MaxAvailableFuelWeight']<0 or not v['Sizing Results']['CouldTrim']:      
          return
      elapsed = 0 # elapsed time since mission start
      fuelAvailable = self.GW - v['Weights']['EmptyWeight'] - v['Weights']['UsefulLoad'] # total fuel weight available, pounds
      w = v['Weights']['EmptyWeight'] + fuelAvailable
      totalFuel = 0.
      totalRange = 0.0
      # step through all the segments specified in the mission file
      for i in range(0, m['NumSegments']):
          if self.debugFine: print "Calculating Mission Segment", i+1
          seg = 'Segment %d' % (i+1)
          # Copy the segment data into the Condition section
          def makeCurrent(section, key, vconfig):
              vconfig['Condition'][key] = section[key]
          m[seg].walk(makeCurrent, vconfig=v)

          w += m[seg]['DeltaLoad'] # add or subtract and weight changes specified in the mission
          if self.debug: print 'Starting at %s    adding load: %d     weight: %d' % (seg, m[seg]['DeltaLoad'], w)
          
          segmentTime = 0.0 # Segment Time Defined in Minutes
          segmentFuel = 0.0 # Segment Fuel Defined in lbs
          disTrav = 0.0     # Distance Traveled defined in notical miles for the segment
          m[seg]['StartTime'] = elapsed # Minutes
          if m[seg]['ClimbSegment']==True:
              currentAlt = m[seg]['StartAltitude'] # Ft
              FinalAlt = m[seg]['Altitude'] # Ft
              while not (currentAlt == FinalAlt) :
                  if self.debugFine: print 'StartTime: %f     Elapsed: %f     Current Alt: %f      Final Alt: %f' % (m[seg]['StartTime'], elapsed, currentAlt, FinalAlt)
                  if self.debugFine: print 'Climb Speed:%f    MaxROC Speed:%f' % (m[seg]['ClimbRate'],m[seg]['Speed'])  
                  v['Condition']['Weight'] = w / m[seg]['IGE Multiplier']
                  # Calculate climb duration and distances
                  duration = v['Simulation']['TimeStep'] # minutes
                  ClimbDis = duration*m[seg]['ClimbRate'] # Find climb distance traveled at this rate (Vertical)(ft)
                  newAlt = currentAlt + ClimbDis
                  if newAlt > FinalAlt:
                      diffAlt = FinalAlt-currentAlt
                      duration = abs(diffAlt/(m[seg]['ClimbRate'])) # minutes
                      ClimbDis = diffAlt # Find climb distance traveled at this rate (Vertical)(ft)
                      newAlt = currentAlt + ClimbDis
                  
                  disTrav += duration*(1./60.) * m[seg]['Speed'] # Find forward distance traveld at this rate (horizontal)(notical miles)
                  
                  # Calculate power required in segment
                  if m[seg]['StartAltitude']>FinalAlt: # Descent Condition
                      TempStore = v['Condition']['ClimbRate']
                      v['Condition']['ClimbRate'] = 0.0
                      (power, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq() # Find resulting fuel consumed
                      v['Condition']['ClimbRate'] = TempStore
                      power *= .75 # Use 75% of level flight power in descent segment.
                                      # This is an estimate from graphs in Sankar's Notes for vortex ring state. also pg 88 Leishman
                                      # This graph is not a great estimate however because it assumes vertical climb and descent and compares
                                      # power to hover power not level flight power. As soon as a better model is found it will be used.
                  elif m[seg]['StartAltitude']==FinalAlt:
                      (power, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq() # Find resulting fuel consumed
                      if self.debugFine: "if it enters in here the input has a mistake, the code will run slower but should run without mistakes"
                  else:
                      (power, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq()
#                      power = m[seg]['95_powerAvail'] # use 95% of available power to climb
                  if math.isnan(power) or power < 0:
                      self.recordTrimFailure()
                      return
                  fuel = self.SFC(power/v['Weights']['NumEngines']) * power * (duration/60.)
                  w -= fuel
                  
                  totalFuel += fuel
                  elapsed += duration
                  currentAlt = newAlt
                  segmentTime += duration
                  segmentFuel += fuel
                  
              totalRange += disTrav
              self.GW = w
              m[seg]['Distance']=disTrav
          elif m[seg]['IdleSegment']>0:
              segmentTime = m[seg]['IdleSegment']
              disTrav = 0.
              power = v['Powerplant']['IdlePower']
              fuel = self.SFC(power/v['Weights']['NumEngines']) * power * segmentTime / 60.0
              w -= fuel
              totalFuel += fuel
              elapsed += segmentTime
              segmentFuel = fuel
              self.GW = w
              totalRange += disTrav
          else:
              while disTrav < m[seg]['Distance']: # keep stepping through until we finish a segment
                  if self.debugFine: print 'StartTime: %f     Elapsed: %f     Distance Traveled: %f      End Distance: %f' % (m[seg]['StartTime'], elapsed, disTrav, m[seg]['Distance'])
                  duration = v['Simulation']['TimeStep'] # Minutes
                  # Check if we're about to overfly the segment
                  stepDis = duration*(1./60.)*m[seg]['Speed'] # notical miles
                  if disTrav+stepDis > m[seg]['Distance']:
                      duration = ((m[seg]['Distance'] - disTrav)/m[seg]['Speed']) # hrs
                      stepDis = duration*m[seg]['Speed'] # notical miles
                      duration *= 60.0 # Conversion to minutes
                      if self.debugFine: print 'Last segment bit, duration %s minutes' % duration
                  disTrav += stepDis
                  v['Condition']['Weight'] = w / m[seg]['IGE Multiplier']
                  (power, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq()
                  if math.isnan(power) or power < 0:
                      self.recordTrimFailure()
                      return
                  fuel = self.SFC(power/v['Weights']['NumEngines']) * power * (duration/60)
                  w -= fuel
                  totalFuel += fuel
                  elapsed += duration
                  segmentTime += duration
                  segmentFuel += fuel
              totalRange += disTrav
              self.GW = w
          if power > m[seg]['95_powerAvail']: m[seg]['Warning Message'] = 'Warning: More Power Required than available'
          m[seg]['Time']= segmentTime
          m[seg]['EndTime'] = elapsed
          m[seg]['Segment Fuel Used'] = segmentFuel
          m[seg]['Total Fuel'] = totalFuel
          if self.debug: print('%s RfR: %.3f' % (seg, totalFuel/self.GW))
      
      self.misSize = fuelAvailable - totalFuel
      if self.debug: print 'Finished!  Total fuel used: %s     Missize amount: %s' % (totalFuel, self.misSize)
      v['Sizing Results']['FuelUsedLb'] = totalFuel
#      v['Sizing Results']['Payload'] = m['Segment 2']['PayloadWeight']
      v['Sizing Results']['FuelUsedGal'] = totalFuel / 6.83
      v['Sizing Results']['TotalWeight'] = v['Weights']['EmptyWeight'] + m['MaxLoad'] + totalFuel
      v['Sizing Results']['MisSize'] = self.misSize
      v['Sizing Results']['TotalRange'] = totalRange
      v['Performance']['MissionRange'] = totalRange
      m['TotalTime'] = elapsed
      m['Mission Fuel Left'] = v['Weights']['MaxAvailableFuelWeight'] - totalFuel
      v['Performance']['MissionRange'] = totalRange
      
      
  def altitudePowerCurve(self):
      ''' Power Requried vs altitude at V best endurance'''
      v = self.vconfig
      v['Condition']['Weight'] = self.GW
      speed = 50. #knots
      powers = []
      altitudes = [0]
      v['Condition']['Density'] = self.density(altitudes[0],v['Engine Scaling']['DeltaTemp'])
      v['Condition']['Speed'] = speed *1.687 #ft/sec
      v['Condition']['ClimbRate'] = 0.0
      
      (totalPower, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq()
      powers.append(totalPower)
      while ((not math.isnan(powers[-1])) or powers[-1]>0) and altitudes[-1] < 30000:
          altitudes.append(altitudes[-1] + 500)
          v['Condition']['Density'] = self.density(altitudes[-1],v['Engine Scaling']['DeltaTemp'])
          print altitudes[-1], v['Condition']['Density']
          (totalPower, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq()
          powers.append(totalPower)
      print "Altitudes:    ", altitudes
      print "Powers:    ", powers
  # Plot Formating
      plt.figure(2)
      plt.plot(altitudes[0:-1],powers[0:-1])
      legend_Data = [v['Condition']['Weight']]
      plt.title('Altitude Power Curve')
      plt.legend((legend_Data), loc=4)
#      plt.axis([0, 200, 0, 15000])
      plt.xlabel('Altitude (ft)')
      plt.ylabel('Power (hp)')
      plt.tight_layout()
      plt.grid(True)
      pylab.savefig('Output/Figures/%s'%v['Veh_Name']['name']+'_AltitudePowerCurve.png', bbox_inches=0, dpi=600)

  def generatePowerCurve(self, altitude):
      v = self.vconfig
      v['Condition']['Weight'] = self.GW
      speeds = [0]
      powersSL = []
      parasite = []
      profile = []
      induced = []
      AdvRatio = []
      CtSigma = []
      avgInflow = []
      coll = []
      beta_0 = []
      theta_1c = []
      theta_1s = []
      TotThrust = []
      alpha_tpp = []
      # Find hover power and start out the arrays
      v['Condition']['Density'] = self.density(altitude,v['Engine Scaling']['DeltaTemp']) # SL
      v['Condition']['Speed'] = speeds[0]
      v['Condition']['ClimbRate'] = 0.0
      (totalPower, Pinduced, Pprofile, Pparasite,TotalThrust,TrimData) = self.powerReq()
      powersSL.append(totalPower)
      induced.append(Pinduced)
      profile.append(Pprofile)
      parasite.append(Pparasite)
      avgInflow.append(TrimData[0])
      coll.append(TrimData[1])
      beta_0.append(TrimData[2])
      theta_1c.append(TrimData[3])
      theta_1s.append(TrimData[4])
      alpha_tpp.append(TrimData[5])
      TotThrust.append(TotalThrust)
      # Do the altitude sweep
      while ((not math.isnan(powersSL[-1])) and (not powersSL[-1]<0)) and speeds[-1]<200:
          
          speed = speeds[-1] + v['Simulation']['PowerCurveResolution']
          v['Condition']['Speed'] = speed
          v['Condition']['Density'] = self.density(altitude,v['Engine Scaling']['DeltaTemp']) # SL
          (totalPower, Pinduced, Pprofile, Pparasite, TotalThrust,TrimData) = self.powerReq()
          powersSL.append(totalPower) # float('nan')
          induced.append(Pinduced)
          profile.append(Pprofile)
          parasite.append(Pparasite)
          speeds.append(speed)
          AdvRatio.append(speed/(v['Main Rotor']['TipSpeed']/1.6878098))
          CtSigma.append(TotalThrust/(v['Condition']['Density']*v['Main Rotor']['DiskArea']*(v['Main Rotor']['Omega']*v['Main Rotor']['Radius'])**2.0*v['Main Rotor']['Solidity']))
          TotThrust.append(TotalThrust)   
          avgInflow.append(TrimData[0])
          coll.append(TrimData[1])
          beta_0.append(TrimData[2])
          theta_1c.append(TrimData[3])
          theta_1s.append(TrimData[4])
          alpha_tpp.append(TrimData[5])
#          print speed        
          if self.debug: print speed
      v['Power Curve']['Speeds'] = speeds
      v['Power Curve']['PowersSL'] = powersSL
      v['Power Curve']['induced'] = induced
      v['Power Curve']['profile'] = profile
      v['Power Curve']['parasite'] = parasite
      v['Power Curve']['Ct Sigma'] = CtSigma
      v['Power Curve']['AdvRatio'] = AdvRatio
      v['Power Curve']['avgInflow'] = avgInflow
      v['Power Curve']['coll'] = coll
      v['Power Curve']['beta_0'] = beta_0
      v['Power Curve']['theta_1c'] = theta_1c
      v['Power Curve']['theta_1s'] = theta_1s
      v['Power Curve']['TotThrust'] = TotThrust
      v['Power Curve']['alpha_tpp'] = alpha_tpp
      
  def findHoverCeiling(self):
      v = self.vconfig
      HCmax = v['Simulation']['HoverCeilingMax']
      HCmin = v['Simulation']['HoverCeilingMin']
      altitude = (HCmax + HCmin) / 2
      steps = 0
      v['Condition']['Weight'] = self.GW
      v['Condition']['Speed'] = 0.
      v['Condition']['ClimbRate'] = 0.
      while steps<v['Simulation']['MaxSteps'] and (HCmax-HCmin)>v['Simulation']['HoverCeilingTolerance'] :
          v['Condition']['Density'] = self.density(altitude, v['Engine Scaling']['DeltaTemp'])
          (powerRequired, Pinduced, Pprofile, Pparasite,_,_) = self.powerReq()
          powerAvailable = self.powerAvailable(altitude)
          if self.debug: pvar(locals(), ('HCmin', 'altitude', 'HCmax', 'powerRequired', 'powerAvailable'))
          if math.isnan(powerRequired) or powerRequired<0 or powerRequired>powerAvailable:
              HCmax = altitude
          else:
              HCmin = altitude
          altitude = (HCmax + HCmin) / 2
      v['Performance']['HoverCeiling'] = altitude

  def density(self, altitude, DeltaTemp):
      # ISA Standard Atmosphere Look up
      pressure = np.interp(altitude, self.alt, self.Pres) * 144.0 # Conversion to lb/ft^2
      temperature = np.interp(altitude,self.alt,self.temp) + 459.67 # Conversion to Rankin
      temperature += DeltaTemp
      R = 1716. # Gas costant Air imperial units (ft lb/slug Rankin)
      rho = pressure/(R*temperature)
      return rho #5e-13*altitude**2 - 7e-8*altitude + .0024 # ISA+15C?(slug/ft^3)

  def powerAvailable(self, altitude):
      v = self.vconfig
      powerAv = v['Powerplant']['MCP'] * self.density(altitude,v['Engine Scaling']['DeltaTemp']) / self.density(0,0)
      return powerAv
      
  def MRPAvailable(self, altitude):
      v = self.vconfig
      powerAv = v['Powerplant']['MRP'] * self.density(altitude,v['Engine Scaling']['DeltaTemp']) / self.density(0,0)
      return powerAv
      
  def SFC(self, power):
      """Returns SFC at a given output power."""
      v = self.vconfig
      sfc = -0.00001495*power + v['Powerplant']['SFC'] - v['Powerplant']['MCP']*(-0.00001495)
      return sfc
      
  def speedOfSound(self, density):
      # look at costello notes, RotorcraftPerformance, p.7 for better equation
      bulkModulusElasticy = 1.01*10.**5. # Pascals, Constant Temp Bulk Modulus
      bulkModulusElasticy = bulkModulusElasticy * .02088547 # (lbf/ft^2)
      c = np.sqrt(1.4*bulkModulusElasticy/density)
      return c

  def findMaxEndurance(self):
      """Finds and stores the maximum enduranve data.  Must be run after generatePowerCurve().
      By nature of the max endurance speed, also finds speed for best rate of climb."""
      v = self.vconfig
      speeds = v['Power Curve']['Speeds']
      powers = v['Power Curve']['PowersSL']
      SPEEDmaxe = 0.
      POWERmaxe = 999999.
      pmin = 9999999.
      for i in range(1, len(speeds)-1):
          if powers[i] < pmin and powers[i] > 0:
              pmin = powers[i]
              SPEEDmaxe = speeds[i]
              POWERmaxe = powers[i]
      if self.debug:  print speeds
      if self.debug:  print powers
      fuelweight = self.GW - v['Weights']['EmptyWeightFraction']*self.GW - v['Weights']['UsefulLoad']

      hourstoempty = fuelweight / (self.SFC(POWERmaxe) * POWERmaxe)
      v['Performance']['SFCatMaxEndurance'] = self.SFC(POWERmaxe)
      v['Performance']['MaxEndurance'] = hourstoempty
      v['Performance']['MaxEnduranceSpeed'] = SPEEDmaxe
      v['Performance']['PowerAtMaxEnduranceSpeed'] = POWERmaxe
      v['Performance']['MaxRateOfClimbSpeed'] = SPEEDmaxe #(knots)
      
  def findMaxClimbRate(self, altitude):
      v = self.vconfig
      PAvailable = self.powerAvailable(altitude)
      v['Performance']['ExcessPowerAvailableAtMaxROCspeed'] = PAvailable-v['Performance']['PowerAtMaxEnduranceSpeed']
      v['Performance']['ClimbSpeed'] = ((v['Performance']['ExcessPowerAvailableAtMaxROCspeed'] / self.GW)*550.)*60.0 # (ft/min)
      if self.debugFine: print 'Power Available:%f    Power Endur Speed:%f     Power Diff:%f' % (PAvailable,v['Performance']['PowerAtMaxEnduranceSpeed'],v['Performance']['ExcessPowerAvailableAtMaxROCspeed'])
      if self.debugFine: print 'Climb Speed:%f     GW: %f' % (v['Performance']['ClimbSpeed'],self.GW)
      
  def findMaxRange(self):
      """Finds and stores the speed for max range and that range.  Must be run after generatePowerCurve()"""
      v = self.vconfig
      speeds = v['Power Curve']['Speeds']
      powers = v['Power Curve']['PowersSL']
      SPEEDmaxr = 0.
      POWERmaxr = 999999.
      imin = 9999999.
      for i in range(1, len(speeds)-1):
          if powers[i]/speeds[i] < imin:
              imin = powers[i]/speeds[i]
              SPEEDmaxr = speeds[i]
              POWERmaxr = powers[i]
      if self.debug:  print speeds
      if self.debug:  print powers
      fuelweight = v['Weights']['MaxAvailableFuelWeight']
      hourstoempty = fuelweight / (self.SFC(POWERmaxr) * POWERmaxr)
      v['Performance']['SFCatMaxRange'] = self.SFC(POWERmaxr)
      v['Performance']['MaxRange'] = hourstoempty * SPEEDmaxr
      v['Performance']['MaxRangeSpeed'] = SPEEDmaxr
      v['Performance']['PowerAtMaxRangeSpeed'] = POWERmaxr

  def findMaxSpeed(self):
      """Finds and stores the maximum speed.  Must be run after generatePowerCurve()"""
      v = self.vconfig
      powerAvailable = self.powerAvailable(0)
      speeds = v['Power Curve']['Speeds']
      powers = v['Power Curve']['PowersSL']
      maxSpeed = 0.
      maxSpeedPower = 0.
      for i in range(len(speeds)-1):
          if powers[i]<powerAvailable and speeds[i]>maxSpeed:
              maxSpeed = speeds[i]
              maxSpeedPower = powers[i]
      v['Performance']['MaxSpeed'] = maxSpeed
      v['Performance']['PowerAtMaxSpeed'] = maxSpeedPower
      v['Performance']['SFCatMaxSpeed'] = self.SFC(maxSpeedPower)

  def scaleEngine(self):
      """Scales the engine for high hot hover and fast cruise."""
      v = self.vconfig
      altitude = v['Engine Scaling']['RequiredHoverHeight']
      DeltaTemp = v['Engine Scaling']['DeltaTemp']
      v['Condition']['Weight'] = self.GW
      v['Condition']['Density'] = self.density(altitude, DeltaTemp)
      v['Condition']['Speed'] = 0 # hover
      v['Condition']['ClimbRate'] = 0.
      hoverpower, Pinduced, Pprofile, Pparasite,_,_ = self.powerReq()
      if math.isnan(hoverpower) or hoverpower<0:
          self.recordTrimFailure()
          hoverpower = 1.
      v['Engine Scaling']['HoverPowerAtAlt'] = hoverpower
      hoverpower = hoverpower * self.density(0,DeltaTemp) / self.density(altitude, DeltaTemp) # scale engine to sea level
      v['Engine Scaling']['HoverPower'] = hoverpower

      ceilingpower = 1.

      if 'Speeds' in v['Power Curve']:
          cruisepower = v['Power Curve']['PowersSL'][-2]
      else:
          cruisepower = 1.
      v['Engine Scaling']['CruisePower'] = cruisepower

      power = max(hoverpower, ceilingpower, cruisepower)
      gamma = power/ v['Weights']['NumEngines'] / self.vconfig['Powerplant']['BaselineMRP']
      sfc = (-0.00932*gamma**2+0.865*gamma+0.445)/(gamma+0.301)*v['Powerplant']['BaselineSFC']
      
      # outputs
      v['Powerplant']['MRP'] = power * 1.3
      v['Powerplant']['MCP'] = power
      v['Powerplant']['SFC'] = sfc * (1-v['Powerplant']['SFCTechImprovementFactor']) # baseline SFC per engine

  def recordTrimFailure(self):
      v = self.vconfig
      v['Sizing Results']['CouldTrim'] = False
      def recordFailure(section, key, vconfig):
          vconfig['Trim Failure'][key] = section[key]
      v['Condition'].walk(recordFailure, vconfig=v)

  def powerReq(self):
      v = self.vconfig
      if v['Main Rotor']['NumRotors'] > 1:
          advancingLiftBalance = .9
      else:
          advancingLiftBalance = .6
      self.rotor.Vtip = v['Main Rotor']['TipSpeed']
      self.rotor.omega = self.rotor.Vtip / v['Main Rotor']['Radius']

      Density = v['Condition']['Density']
      V = v['Condition']['Speed'] * 1.687 # speed in ft/sec
      ClimbRate = v['Condition']['ClimbRate']/60.0 # Ft/sec
      
      # proportion out the vertical lift on the rotors
      VerticalLift_rotors = v['Condition']['Weight']
      VerticalLift_perRotor = VerticalLift_rotors / v['Main Rotor']['NumRotors']
 
      # proportion out forward thrust between the aux prop and the rotors
      BodyDrag = .5 * Density * V**2 * v['Body']['FlatPlateDrag']
      TotalDrag = BodyDrag
      ForwardThrust = TotalDrag

      ForwardThrust_rotors = ForwardThrust
      ForwardThrust_perRotor = ForwardThrust_rotors / v['Main Rotor']['NumRotors']
      if self.debug: pvar(locals(), ('ForwardThrust_perRotor', 'VerticalLift_perRotor'))

      # calculate rotor power
      (singleRotorPower, Pinduced, Pprofile, TrimData) = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], \
                        V=V, rho=Density, speedOfSound=self.speedOfSound(Density), Fx=ForwardThrust_perRotor, \
                        Fz=VerticalLift_perRotor, maxSteps=v['Simulation']['MaxSteps'], \
                        advancingLiftBalance=advancingLiftBalance, returnAll=True, Vcl=ClimbRate)
      
      if singleRotorPower>0: singleRotorPower = singleRotorPower / (1-v['Body']['DownwashFactor']) * v['Main Rotor']['Kint']  # REMOVEME # fix Kint?
#      print singleRotorPower      
      TotalThrust = np.sqrt(ForwardThrust_rotors**2+VerticalLift_rotors**2)
      # Calculate anti torque power
      if (not math.isnan(singleRotorPower)) and (singleRotorPower>0) and v['Main Rotor']['NumRotors']==1:
          PAntitorque = self.findAntiTorquePower(singleRotorPower,Density,V)
      else:
          PAntitorque = 0#float('nan')
      
           
      if self.debug: pvar(locals(), ('BodyDrag','VerticalLift_perRotor', 'singleRotorPower'))
      # find total power
      if singleRotorPower > 0:
        totalPower = singleRotorPower*v['Main Rotor']['NumRotors'] +TotalDrag*V/550.0 # Is this right?  should the parasite power be just added on directly like this?
        totalPower = (totalPower + PAntitorque) / (1-v['Powerplant']['TransmissionEfficiency'])
      else:
        totalPower = singleRotorPower
      
      v['Performance']['MaxBladeLoadingSeen'] = max(v['Performance']['MaxBladeLoadingSeen'], math.sqrt(ForwardThrust_perRotor**2+VerticalLift_perRotor**2)/(Density*v['Main Rotor']['DiskArea']*self.rotor.Vtip**2))

      Pparasite = TotalDrag*V/550.0
      alpha_tpp = math.atan(ForwardThrust_perRotor/VerticalLift_perRotor)
      TrimData.append(alpha_tpp*180/math.pi)
      return (totalPower, Pinduced, Pprofile, Pparasite, TotalThrust, TrimData)
      
  def findAntiTorquePower(self,Tpower,density,Vinf):
      """AntiTorque Power is calculated using Momentum Theory forward flight corrections """
      v = self.vconfig
      reqTor = Tpower/v['Main Rotor']['Omega']*550.0 # lb*ft
      TailLength = v['Antitorque']['TailLength_RotorRadiusRatio']*v['Main Rotor']['Radius']
      ThrustTail = reqTor/TailLength #lbs
      TArea = v['Antitorque']['DiskArea']
      VT = v['Antitorque']['TipSpeed']
      CD0 = v['Antitorque']['CD0']
      sigma = v['Antitorque']['TailSolidity']

      v = np.sqrt(ThrustTail/(2*density*TArea)) # Induced Velocity
      V_v = Vinf/v

      mu = Vinf/VT

      Ku = np.sqrt((-V_v**2.0+np.sqrt(V_v**4.0 + 4.0))/2.0)
      Induced = (1.13*ThrustTail*v*Ku)/550
      Profile = (CD0*density*TArea*sigma*VT**3.0)/(550.0*8.0)*(1.0 + 4.65*mu)
      Ptotal = Induced+Profile
      return (Ptotal)
      
  def sizeAntiTorque(self):
    """This code sizes the antitorque rotor for input knots"""
    v = self.vconfig
    if v['Main Rotor']['NumRotors'] > 1:
          advancingLiftBalance = .9
    else:
      advancingLiftBalance = .6
    self.rotor.Vtip = v['Main Rotor']['TipSpeed']
    self.rotor.omega = self.rotor.Vtip / v['Main Rotor']['Radius']############################################# 
    
    
    
    V = v['Antitorque']['SizingSpeed'] * 1.687 # ft/sec
    density = self.density(v['Engine Scaling']['CruiseAltitude'],v['Engine Scaling']['DeltaTemp'])
    
    BodyDrag = .5 * density * V**2 * v['Body']['FlatPlateDrag']
    TotalDrag = BodyDrag
    ForwardThrust = TotalDrag

    ForwardThrust_rotors = ForwardThrust
    ForwardThrust_perRotor = ForwardThrust_rotors / v['Main Rotor']['NumRotors']
#    ForwardThrust_perRotor = 0.
    
    VerticalLift_rotors = self.GW
    VerticalLift_perRotor = VerticalLift_rotors / v['Main Rotor']['NumRotors']
    advancingLiftBalance = .5
    (Tpower, _, _, _) = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], \
                        V=V, rho=density, speedOfSound=self.speedOfSound(density), Fx=ForwardThrust_perRotor, \
                        Fz=VerticalLift_perRotor, maxSteps=v['Simulation']['MaxSteps'], \
                        advancingLiftBalance=advancingLiftBalance, returnAll=True, Vcl = 0.0)
    if Tpower<0:
      v['Antitorque']['DiskArea'] = 9999
    else:
      reqTor = Tpower/v['Main Rotor']['Omega']*550.0 # lb*ft
      TailLength = v['Antitorque']['TailLength_RotorRadiusRatio']*v['Main Rotor']['Radius']
      ThrustTail = reqTor/TailLength #lbs
      
#      print ThrustTail, v['Antitorque']['TailDiskLoading'], Tpower, self.GW
      v['Antitorque']['Radius'] = math.sqrt(ThrustTail / (math.pi * v['Antitorque']['TailDiskLoading']))
      v['Antitorque']['DiskArea'] = math.pi*v['Antitorque']['Radius']**2
    
  def Ct_SigmaCurve(self):
      # Takes in flight conditions returns max thrust at that condition by finding
      # whether the AC can trim or not at that condition. Uses bisecting to find
      # max thrust. Will take in forward speed, density, any other necessary info...
      v = self.vconfig
      if v['Main Rotor']['NumRotors'] > 1:
        advancingLiftBalance = .9
      else:
        advancingLiftBalance = .5
      self.rotor.Vtip = v['Main Rotor']['TipSpeed']
      self.rotor.omega = self.rotor.Vtip / v['Main Rotor']['Radius']
      altitude = v['Simulation']['Curve_Altitude']
      Density = self.density(altitude,v['Engine Scaling']['DeltaTemp'])
      v['Condition']['Weight'] = self.GW
      speed = 0.0 # Knots
      speeds = [0.0] #knots
      powersSL = [0.0]
      MaxThrusts = []
      CtSigma = []
      AdvRatio = []
      while ((not math.isnan(powersSL[-1])) or (not powersSL[-1]<0)) and speed<=200:
        V = speed * 1.687 # ft/sec
        error = 1000.
        # proportion out the vertical lift on the rotors
        VLiftLower = 0#v['Condition']['Weight']
        VLiftUpper = v['Condition']['Weight']*50
        steps = 0
        print "CT_Sigma V (knots)= ", speed
#        print "error = ", error
#        print "tollerance = ", v['Simulation']['CT_SigmaTolerance']
        while steps<v['Simulation']['MaxSteps'] and error>v['Simulation']['CT_SigmaTolerance']:
          VerticalLift_rotors = (VLiftLower+VLiftUpper)*.5
          VerticalLift_perRotor = VerticalLift_rotors / v['Main Rotor']['NumRotors']
          # proportion out forward thrust between the aux prop and the rotors
          BodyDrag = .5 * Density * V**2 * v['Body']['FlatPlateDrag']
          TotalDrag = BodyDrag
          ForwardThrust = TotalDrag
  
          ForwardThrust_rotors = ForwardThrust
          ForwardThrust_perRotor = ForwardThrust_rotors / v['Main Rotor']['NumRotors']
          if self.debug: pvar(locals(), ('ForwardThrust_perRotor', 'VerticalLift_perRotor'))
  
          # calculate rotor power
          (singleRotorPower, Pinduced, Pprofile, _) = self.rotor.trim(tolerancePct=v['Simulation']['TrimAccuracyPercentage'], \
                            V=V, rho=Density, speedOfSound=self.speedOfSound(Density), Fx=ForwardThrust_perRotor, \
                            Fz=VerticalLift_perRotor, maxSteps=v['Simulation']['MaxSteps'], \
                            advancingLiftBalance=advancingLiftBalance, returnAll=True, Vcl = 0.0)
          
          if math.isnan(singleRotorPower) or singleRotorPower<0:
            VLiftUpper = VerticalLift_rotors
          else:
            VLiftLower = VerticalLift_rotors
          error = (VLiftUpper-VLiftLower)
          steps += 1 
#          print "Weight = ", VerticalLift_perRotor, "Power Required = ", singleRotorPower, "Lift Balance =",advancingLiftBalance, "\n"
        powersSL.append(singleRotorPower*v['Main Rotor']['NumRotors']) # HP
        MaxThrust = np.sqrt(ForwardThrust_rotors**2+VerticalLift_rotors**2) # lbs
        CtSigma.append(MaxThrust/(Density*v['Main Rotor']['DiskArea']*(v['Main Rotor']['Omega']*v['Main Rotor']['Radius'])**2.0*v['Main Rotor']['Solidity']))
        AdvRatio.append(speed/(v['Main Rotor']['TipSpeed']/1.6878098)) # no Unit
        speed = speeds[-1] + v['Simulation']['CT_SigmaCurveResolution'] # knots
        speeds.append(speed) #knots
      v['Power Curve']['Ct Sigma Max'] = CtSigma
      v['Power Curve']['Ct Sigma AdvRatio'] = AdvRatio
  
  
  def write(self):
      # write out the output
      v = self.vconfig
      'Config/C81/%s'%v['Main Rotor']['AirfoilFile']
      v.filename = 'Output/%s'%v['Veh_Name']['name']+'_VehOutput.cfg'
      print('writing')
      v.write()
      print('written')
  
      m = self.mconfig
      m.filename = 'Output/%s'%v['Veh_Name']['name']+'_MissionOut.cfg'
      m.write()