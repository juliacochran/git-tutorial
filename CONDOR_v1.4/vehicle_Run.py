# -*- coding: utf-8 -*-
"""
Created on Fri Oct 24 13:49:16 2014

@author: bengland3
"""


from configobj import ConfigObj
from validate import Validator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from vehicle import Vehicle

def run_vehicle(Master):

    v = ConfigObj(Master['Veh Mission Config']['Aircraft_Config'], configspec='Config/Configspec/vehicle.configspec')
    m = ConfigObj(Master['Veh Mission Config']['Mission_Config'], configspec='Config/Configspec/mission.configspec')
    vvdt = Validator()
    v.validate(vvdt)
    mvdt = Validator()
    m.validate(mvdt)
    c81File_mainRotor = 'Config/C81/%s' % v['Main Rotor']['AirfoilFile']
    airfoildata_mainRotor = np.genfromtxt(c81File_mainRotor, skip_header=0, skip_footer=0) # read in the airfoil file
    plt.figure(num=None, figsize=(6, 4), facecolor='w', edgecolor='k')
    GW0 = v['Weights']['BaselineGrossWeight']
    if Master['Vehicle Options']['vehicle_size_compare']:
        GW_Vals = [.8*GW0,.9*GW0, GW0]
    else:
        GW_Vals = [GW0]
    for GW in GW_Vals:
        print "Performance Calculations for GW=", GW
        vehicle = Vehicle(v, m, GW, airfoildata_mainRotor, Master)
        vehicle.generatePowerCurve(v['Simulation']['Curve_Altitude'])
        vehicle.findMaxRange()
        vehicle.findMaxEndurance()
        vehicle.findMaxSpeed()

        if Master["Vehicle Options"]["vehicle_Alt_Power_Curve"]:
          vehicle.altitudePowerCurve()
        
        plt.figure(1)   
        plt.plot(vehicle.vconfig['Power Curve']['Speeds'], vehicle.vconfig['Power Curve']['PowersSL'])
        if Master["Vehicle Options"]["Power_Breakdown"]:
              plt.plot(vehicle.vconfig['Power Curve']['Speeds'], vehicle.vconfig['Power Curve']['induced'])
              plt.plot(vehicle.vconfig['Power Curve']['Speeds'], vehicle.vconfig['Power Curve']['profile'])
              plt.plot(vehicle.vconfig['Power Curve']['Speeds'], vehicle.vconfig['Power Curve']['parasite'])
#        print vehicle.vconfig['Power Curve']['PowersSL']
        
        if Master["Vehicle Options"]["vehicle_CT_Sigma_Curve"]: 
          vehicle.Ct_SigmaCurve()
          plt.figure(3)
          plt.plot(vehicle.vconfig['Power Curve']['AdvRatio'], vehicle.vconfig['Power Curve']['Ct Sigma'])
          plt.plot(vehicle.vconfig['Power Curve']['Ct Sigma AdvRatio'], vehicle.vconfig['Power Curve']['Ct Sigma Max'])
          plt.title(v['Veh_Name']['name']+' CT/sigma vs Advance Ratio: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = ' + str(GW))

        
        vehicle.flyMission() # This must come after the other ones becuase it changes the GW
        vehicle.write()

# Begin Plotting Sections, No more actual vehicle function funs after this.      
        
        if Master["Vehicle Options"]["Trim_Velocity_Plots"]:
          end = np.size(vehicle.vconfig['Power Curve']['Speeds']) # -1 #
          plt.figure(4)
#          print np.size(vehicle.vconfig['Power Curve']['Speeds']), np.size(vehicle.vconfig['Power Curve']['avgInflow'])
          plt.plot(vehicle.vconfig['Power Curve']['Speeds'][0:end],vehicle.vconfig['Power Curve']['avgInflow'][0:end])
          plt.title(v['Veh_Name']['name']+' Average Inflow: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = '+ str(GW))
          plt.xlabel('Speed (kts)')
          plt.ylabel('ft/sec')          
          plt.tight_layout()
          plt.grid(True)
          pylab.savefig('Output/Figures/Detailed/%s'%v['Veh_Name']['name']+'_AvgInflow.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
          
          plt.figure(5)
          plt.plot(vehicle.vconfig['Power Curve']['Speeds'][0:end],vehicle.vconfig['Power Curve']['coll'][0:end])
          plt.title(v['Veh_Name']['name']+' Collective Angle: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = ' + str(GW))
          plt.xlabel('Speed (kts)')
          plt.ylabel('Degrees')          
          plt.tight_layout()
          plt.grid(True)
          pylab.savefig('Output/Figures/Detailed/%s'%v['Veh_Name']['name']+'_Collective.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
          
          plt.figure(6)
          plt.plot(vehicle.vconfig['Power Curve']['Speeds'][0:end],vehicle.vconfig['Power Curve']['beta_0'][0:end])
          plt.title(v['Veh_Name']['name']+' beta 0 angles: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = ' + str(GW))
          plt.ylabel('degrees')
          plt.xlabel('Speed (kts)')          
          plt.tight_layout()
          plt.grid(True)
          pylab.savefig('Output/Figures/Detailed/%s'%v['Veh_Name']['name']+'_Beta_0.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
          
          plt.figure(7)
          plt.plot(vehicle.vconfig['Power Curve']['Speeds'][0:end],vehicle.vconfig['Power Curve']['theta_1c'][0:end])
          plt.title(v['Veh_Name']['name']+' Theta 1c: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = ' + str(GW))
          plt.ylabel('degrees')
          plt.xlabel('Speed (kts)')          
          plt.tight_layout()
          plt.grid(True)
          pylab.savefig('Output/Figures/Detailed/%s'%v['Veh_Name']['name']+'_Theta_1c.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
          
          plt.figure(8)
          plt.plot(vehicle.vconfig['Power Curve']['Speeds'][0:end],vehicle.vconfig['Power Curve']['theta_1s'][0:end])
          plt.title(v['Veh_Name']['name']+' Theta 1s: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = ' + str(GW))
          plt.ylabel('degrees')
          plt.xlabel('Speed (kts)')          
          plt.tight_layout()
          plt.grid(True)
          pylab.savefig('Output/Figures/Detailed/%s'%v['Veh_Name']['name']+'_Theta_1s.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
          
          plt.figure(9)
          plt.plot(vehicle.vconfig['Power Curve']['Speeds'][0:end],vehicle.vconfig['Power Curve']['alpha_tpp'][0:end])
          plt.title(v['Veh_Name']['name']+' alpha_tpp: ' + str(v['Simulation']['Curve_Altitude']) + ' ft' + ' GW = ' + str(GW))
          plt.ylabel('degrees')
          plt.xlabel('Speed (kts)')          
          plt.tight_layout()
          plt.grid(True)
          pylab.savefig('Output/Figures/Detailed/%s'%v['Veh_Name']['name']+'_alpha_tpp.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
        


    MCPspeeds = [0, 300]
    MCP = [vehicle.powerAvailable(v['Simulation']['Curve_Altitude']), vehicle.powerAvailable(v['Simulation']['Curve_Altitude'])]
    MRP = [vehicle.MRPAvailable(v['Simulation']['Curve_Altitude']),vehicle.MRPAvailable(v['Simulation']['Curve_Altitude'])]
    plt.figure(1)
    plt.plot(MCPspeeds, MCP)
    plt.plot(MCPspeeds, MRP)
# Data for power curve Verification
    vmax = 0.0
    pmax = 0.0
    # S92 Data
    if v['Veh_Name']['name']=='s92':
        s92_V = np.array([50.1073, 60.2146, 69.9122, 80.0195, 90.1268, 100.098, 110.205, 120.312, 127.415, 137.249, 144.624, 151.59, 156.644, 163.2, 167.98])
        s92_fuel = np.array([1240.51, 1156.96, 1113.92, 1096.2, 1103.8, 1129.11, 1177.22, 1240.51, 1301.27, 1410.13, 1518.99, 1655.7, 1772.15, 1972.15, 2134.18])
        s92_HP = 6.68896*(2452-np.sqrt(6012300-1495*s92_fuel))
        plt.figure(1)
        plt.plot(s92_V,s92_HP)
        vmax = max(s92_V)
        pmax = max(s92_HP)
    # XH-59
    elif v['Veh_Name']['name']=='XH59':
        xh59_V = np.array([0, 21.966997, 42.561056, 63.471947, 85.122112, 87.762376, 89.663366, 91.669967, 105.927393, 108.567657, 120.712871, 125.887789, 126.627063, 130.112211, 136.765677, 136.554455, 143.841584, 146.481848, 151.339934, 154.719472])
        xh59_HP = np.array([1414.285714, 1168.163265, 925.714286, 813.673469, 732.857143, 800.816327, 854.081633, 786.122449, 879.795918, 817.346939, 912.857143, 1129.591837, 1050.612245, 1002.857143, 1054.285714, 1206.734694, 1307.755102, 1434.489796, 1520.816327, 1647.55102])
        plt.figure(1)        
        plt.plot(xh59_V,xh59_HP)
        vmax =max(xh59_V)
        pmax =max(xh59_HP)
    # Boeing AH6G/ MD 530FF
    elif v['Veh_Name']['name']=='MD_530FF':
        AH6G_V = np.array([30,35,40,45,50,55,60,65,70,75,80,85,90,95,100])
        AH6G_HP = np.array([295,270,255,245,240,240,245,250,260,275,290,310,335,365,390])
        plt.figure(1)        
        plt.plot(AH6G_V,AH6G_HP)
        vmax = max(AH6G_V)
        pmax = max(AH6G_HP)
    # Sikorsky CH53E
    elif v['Veh_Name']['name']=='CH53E':
        CH53E_V= np.array([0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160])
        CH53E_HP=np.array([.18,.178,.17,.152,.13,.115,.1,.095,.09,.088,.09,.091,.094,.098,.105,.11,.125])*GW0
        plt.figure(1)
        plt.plot(CH53E_V,CH53E_HP)
        vmax = max(CH53E_V)
        pmax = max(CH53E_HP)
    # Aerospatial AS365N
#    AS365_V
#    AS365_HP
    
    
# Plot Formating
    plt.figure(1)
    types=['Induced: ','Profile: ','Parasite: ']
    if Master["Vehicle Options"]["Power_Breakdown"]:
      data = [[j+str(i)+'lbs' for j in types] for i in GW_Vals][0]
      legend_Data = [str(i)+'lbs' for i in GW_Vals]+[i for i in data]+["MCP"]+["MRP"]+[v['Veh_Name']['name']]
    else:
      legend_Data = [str(i)+'lbs' for i in GW_Vals]+["MCP"]+["MRP"]+[str(i)+'lbs' for i in GW_Vals]+[v['Veh_Name']['name']]  
    
    
    plt.title(v['Veh_Name']['name']+' Power Curve: ' + str(v['Simulation']['Curve_Altitude']) + ' ft')
    plt.legend((legend_Data),  loc=2, fontsize = 'small')
    plt.axis([0, max([vmax,max(vehicle.vconfig['Power Curve']['Speeds'])]), 0, 1.5 * max([pmax,max(vehicle.vconfig['Power Curve']['PowersSL'])])])
    plt.xlabel('Speed (kts)')
    plt.ylabel('Power (hp)')
    plt.tight_layout()
    plt.grid(True)
    pylab.savefig('Output/Figures/%s'%v['Veh_Name']['name']+'_PowerCurveCruise.png', bbox_inches=0, bbox_extra_artists=True, dpi=1000)
    
    if Master["Vehicle Options"]["vehicle_CT_Sigma_Curve"]:
      plt.figure(3)
      legend_Data = [str(i)+'lbs' for i in GW_Vals]
      plt.title(v['Veh_Name']['name'] + ' Ct/Sigma vs Advance Ratio: '+ str(v['Simulation']['Curve_Altitude']) + ' ft')
      plt.legend((legend_Data), loc='upper left', bbox_to_anchor=(1,1))
      plt.axis([0,max(vehicle.vconfig['Power Curve']['AdvRatio']+vehicle.vconfig['Power Curve']['Ct Sigma AdvRatio']),0,2*max(vehicle.vconfig['Power Curve']['Ct Sigma']+vehicle.vconfig['Power Curve']['Ct Sigma Max'])],)
      plt.xlabel('Advance Ratio')
      plt.ylabel('Ct/Sigma')
      plt.tight_layout()
      plt.grid(True)
      pylab.savefig('Output/Figures/%s'%v['Veh_Name']['name']+'_CtSigma_VS_AdvRatio.png', bbox_inches=0, dpi=1000)
    
    
    if Master['Vehicle Options']['vehicle_showPower']: 
        plt.show()
    else:
        plt.close('all')

def pvar(locals_, vars_):
    s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)