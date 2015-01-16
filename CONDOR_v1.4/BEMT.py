# CONDOR
# Integrated Product Lifecylce Engineering Labratory


import math
import time
import numpy as np

def pvar(locals_, vars_):
    '''Prints the variables and there name'''
    s = ['%s: %.3f' % (var, locals_[var]) for var in vars_]
    print '     '.join(s)

def pdeg(locals_, vars_):
    s = ['%s: %.2f' % (var*180./math.pi, locals_[var]) for var in vars_]
    print '     '.join(s)


class Blade:
    # This is a very simple class intended to just hold the configuration for a single blade.
    # Broken down into radial pieces, and with the c81 file read in and interpolation splines
    # generated so that cl, cd, etc. can be returned easily later.  It will contain a
    # non-dimensional blade, with everything referenced to a radius of 1.

    def __init__(self, airfoildata, averageChord, Master, skip_header=0, skip_footer=0,\
                 taperRatio=1, tipTwist=0, rootCutout=.1, segments=15, \
                 dragDivergenceMachNumber=.85):
        self.dragDivergenceMachNumber = dragDivergenceMachNumber
        self.rootCutout = rootCutout
        # store the airfoil data.  They are assumed to be in degrees, and are converted to radians
        self.alphadata = airfoildata[:,0]*math.pi/180
        self.cldata = airfoildata[:,1]
        self.cddata = airfoildata[:,2]
        self.cmdata = airfoildata[:,3]
        # generate the piecewise data on the blade
        endpoints, self.dr = np.linspace(rootCutout, 1, segments+1, retstep=True)
        self.r = np.zeros(endpoints.size - 1)
        # set the r values as the midpoints of each segment
        for i in range(self.r.size):
            self.r[i] = (endpoints[i] + endpoints[i+1]) / 2
        # create the splines and store the twist and chord distribution
        self.twist = np.interp(self.r, [rootCutout,1], [0,tipTwist*math.pi/180.])
        twist75 = np.interp(.75, [rootCutout,1], [0,tipTwist*math.pi/180.])
        self.twist -= twist75
        self.theta_tw = tipTwist * math.pi/180.
        rootChord = 2 * averageChord / (taperRatio + 1)
        self.chord = np.interp(self.r, [rootCutout,1], [rootChord,rootChord*taperRatio])
        self.debug = Master['BEMT Options']['BEMT_debug']
        self.plot = Master['BEMT Options']['BEMT_plot']
        self.animate =Master['BEMT Options']['BEMT_animate']

    def cl(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        cls = np.interp(alpha, self.alphadata, self.cldata)
        cls = np.reshape(cls, shape)
        return cls

    def cd(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        cds = np.interp(alpha, self.alphadata, self.cddata)
        cds = np.reshape(cds, shape)
        return cds

    def cm(self, alpha):
        shape = alpha.shape
        alpha = np.reshape(alpha, alpha.size)
        cms = np.interp(alpha, self.alphadata, self.cmdata)
        cms = np.reshape(cms, shape)
        return cms

    def find_alpha(self, desiredCl):
        alphas = np.arange(-math.pi/12, math.pi/4, 0.01745)
        Cl = self.cl(alphas)
        alpha = math.pi / 12 # we'll just return 15 degrees if we can't find it.  Probably won't be able to trim if it's not found, but this way we can try anyway
        for i in xrange(Cl.size):
            if Cl[i] > desiredCl:
                alpha = alphas[i]
                break
        return alpha

    def write(self):
        # right now this function just writes out some of the blade data to test the reading and interpolation routines.  Should remove it once the class is finalized.
        np.savetxt('Config/Blade.csv', np.transpose((self.r, self.theta)))
        alphas = np.arange(15)
        cl = self.cl(alphas)
        cd = self.cd(alphas)
        cm = self.cm(alphas)
        np.savetxt('Config/Airfoil.csv', np.transpose((alphas, cl, cd, cm)))


class Rotor:
    """This class holds the rotor and actually deals with trimming it and whatnot.

    It contains dimensional quantities."""

    def __init__(self, blade, psiSegments, Vtip, radius, numblades, Master):
        self.blade = blade
        self.Vtip = Vtip
        self.radius = radius
        self.R = radius
        self.numblades = numblades
        self.omega = Vtip / radius
        # set up 1D arrays of psi and r...
        self.psi1D, self.dpsi = np.linspace(0, 2*math.pi, psiSegments, endpoint=False, retstep=True)
        r1D = self.blade.r
        # ...and convert them to 2D, plus chord and theta
        self.r, self.psi = np.meshgrid(r1D, self.psi1D)
        self.chord, self.psi = np.meshgrid(blade.chord, self.psi1D)
        # dimensionalize some of the blade data
        self.chord = self.chord * radius
        self.r = self.r * radius
        self.dr = self.blade.dr * radius
        # calculate the solidity
        self.bladeArea = np.sum(self.chord) * self.dr * self.dpsi * numblades / (2*math.pi)
        self.diskArea = math.pi * radius**2
        self.solidity = self.bladeArea / self.diskArea
        # precompute and store the values of sin(psi) and cos(psi) since they're static and we're going to use them a lot later on
        self.cospsi = np.cos(self.psi)
        self.sinpsi = np.sin(self.psi)
        # initialize the trim variables such that they'll get set when we first try to trim
        self.beta_0 = 9999999.
        self.beta_1c = 9999999.
        self.beta_1s = 9999999.
        self.theta_0 = 9999999.
        self.theta_1c = 9999999.
        self.theta_1s = 9999999.
        self.inflow = 9999999.
        self.power = np.nan
        self.debug = Master['BEMT Options']['BEMT_debug']
        self.plot = Master['BEMT Options']['BEMT_plot']
        self.animate = Master['BEMT Options']['BEMT_animate']

    def reinitializeTrimVars(self):
        # We'll just use .1 rad = 5.73 deg to initialize for now.  It'd be better to use closed-form estimates, but we'll add that later.
        self.beta_0 = .1 # precone
        self.theta_0 = .2
        self.theta_1c = 0.
        self.theta_1s = 0.



    # probably obsolete now!  will delete as soon as sure.
    def calcInflow(self, Fx, Fz, rho, V, maxSteps, Vcl = 0.0):
        # V = Forward Velocity
        # Need to add climb rate = Vcl
        # inflow calculations
        alpha_TPP = math.atan(Fx/Fz)
        thrust = math.sqrt(Fx**2 + Fz**2)
        diskArea = math.pi * self.R**2
        inflow = math.sqrt(thrust / (2.*rho*diskArea))
        inflow_avg = inflow
        change = 999.
        steps = 0
        # I believe these inflow equations are only valid for small alpha_TPP, so are not valid for propellers
        while change>.00001 and steps<maxSteps:
            # Root/Tip loss model by Prandlt (documented in Leishman pg 141 )
            # Just the tip
            f = self.numblades/2. * (1-self.r/self.R)/(inflow_avg/self.Vtip)
            F = 2./math.pi * np.arccos(np.exp(-f))
            # Now the root
            f = self.numblades/2. * (self.r/self.R-self.blade.rootCutout)/(inflow_avg/self.Vtip)
            F *= 2./math.pi * np.arccos(np.exp(-f))
            #F[~np.isfinite(F)] = 0.001
#            newinflow = thrust / (2.*rho*diskArea*np.sqrt((V*math.cos(alpha_TPP))**2 + (V*math.sin(alpha_TPP) + F*inflow)**2))
#            # The next lines are a proposed alteration to include climb speed
#           
            newinflow = thrust / (2.*rho*diskArea*np.sqrt((V*math.cos(alpha_TPP) + Vcl*math.sin(alpha_TPP))**2 \
                        + (V*math.sin(alpha_TPP) + F*inflow + Vcl* math.cos(alpha_TPP))**2))
            
            change = abs(np.sum(newinflow) - np.sum(inflow)) / np.sum(inflow)
            inflow = newinflow
            inflow_avg = inflow.sum() / inflow.size
            #print inflow[-1][-1]
            steps += 1

            
        #uniform = (inflow * self.r) / self.r
        self.F = F
        if alpha_TPP < math.pi/4.:
#          mu_x = V*math.cos(alpha_TPP)/(self.omega*self.R)
          mu_x = (V*math.cos(alpha_TPP)+Vcl*math.sin(alpha_TPP))/(self.omega*self.R) # Potential Change for climbing flight
#          mu_z = V*math.sin(alpha_TPP)/(self.omega*self.R)
          k_x = (4/3)*mu_x/inflow/(1.2+mu_x/inflow) # Payne model (Leishman 160)
          k_y = 0 # Payne model (Lesihman 160)
          
#          k_x = mu_x/(2.*(mu_z+inflow)) # Coleman model (Leishman 160)
#          k_y = 0. # Coleman model (Leishman 160)
#          Xi = np.arctan(mu_x/(mu_z+inflow))
#          k_x = (4/3)*(1. - np.cos(Xi) - 1.8*mu_x**2)/np.sin(Xi) # Drees Model (Leishman 160) [DOESN'T WORK: CODE FAILS]
#          k_y = -2. * mu_x # Drees Model (Leishman 160) [DOESN'T WORK: CODE FAILS]
          
# Glauert non-uniform inflow correction          
          inflow = inflow * (1. + k_x*self.r*self.cospsi + k_y*self.r*self.sinpsi)
          
          
#          print np.sum(inflow)/ np.size(inflow)
        # import matplotlib.pyplot as plt
        # fig = plt.figure()
        # fig.add_subplot(221, projection='polar')
        # c = plt.contourf(self.psi, self.r, f)
        # plt.colorbar(c)
        # plt.title('f')
        # fig.add_subplot(222, projection='polar')
        # c = plt.contourf(self.psi, self.r, F)
        # plt.colorbar(c)
        # plt.title('F')
        # fig.add_subplot(223, projection='polar')
        # c = plt.contourf(self.psi, self.r, inflow)
        # plt.colorbar(c)
        # plt.title('inflow')
        # fig.add_subplot(224, projection='polar')
        # c = plt.contourf(self.psi, self.r, uniform)
        # plt.colorbar(c)
        # plt.title('uniform')
        # plt.show()
#        print "Inflow=", inflow
        return inflow
        

    def trimTandem(self, tolerancePct, V, speedOfSound, rho, Fx, Fz, maxSteps):
        liftBalanceMin = .4
        liftBalanceMax = 1.
        powers = [0, 0 ,0]
        powers[0] = self.trim(tolerancePct, V, speedOfSound, rho, Fx, Fz, maxSteps, advancingLiftBalance=liftBalanceMin)
        powers[2] = self.trim(tolerancePct, V, speedOfSound, rho, Fx, Fz, maxSteps, advancingLiftBalance=liftBalanceMax)

    def trim(self, tolerancePct, V, speedOfSound, rho, Fx, Fz, maxSteps, advancingLiftBalance=.5, returnAll=False, Vcl=0.0):
        """Attempts to trim the rotor at given conditions.  Will re-use trim variables from last time if possible.
        Note that "drag" and "weight" in the variables are really just the veritcal and horizontal components of
        the force that the rotor is generating."""

        if self.animate or self.plot:
            import matplotlib.pyplot as plt

        psi = self.psi
        r = self.r
        R = self.R
        chord = self.chord
        omega = self.omega
        dr = self.dr
        dpsi = self.dpsi
        numblades = self.numblades
        cospsi = self.cospsi
        sinpsi = self.sinpsi
        Vtip = self.Vtip
        blade = self.blade
        rootCutout = blade.rootCutout * R
        lockNumber = 8. # assumed estimate
        alpha_TPP = math.atan(Fx/Fz) # tip path plane angle
        totalThrust = math.sqrt(Fx**2 + Fz**2)

        if self.debug: pvar(locals(), ('V', 'Fx', 'Fz'))

        # First we'll check if the pre-existing trim solution is reasonable. If not, re-initialize them.
        if (self.beta_0<0) or (self.beta_0>math.pi/6) or (abs(self.theta_1c)>math.pi/6) or (abs(self.theta_1s)>math.pi/6) or math.isnan(self.power):
            self.reinitializeTrimVars()
            #self.inflow = np.ones(r.shape) * math.sqrt(math.sqrt(Fx**2 + Fz**2) / (2.*rho*self.diskArea))
        self.inflow = self.calcInflow(Fx=Fx, Fz=Fz, rho=rho, V=V, maxSteps=maxSteps, Vcl = Vcl)

        #F = self.F
        beta_0 = self.beta_0
        theta_0 = self.theta_0
        theta_1c = self.theta_1c
        theta_1s = self.theta_1s
        inflow = self.inflow
        furtherLastInflow = inflow
        lastInflow = inflow
        inflow = inflow
        roll = 999
        pitch = 999
        L = 0
        P = 1
        T = 0
        lastP = 0
        secondLastP = 0
        steps = 0
        CChanges = []
        TChanges = []
        theta_0_hist = []
        b0_hist = []
        t1c_hist = []
        t1s_hist = []
        thrust_hist = []
        roll_hist = []
        pitch_hist = []
        miscA_hist = []
        miscB_hist = []
        F = np.zeros(r.shape)
        dT = np.zeros(r.shape)
        dL = np.zeros(r.shape)
        alphaEffective = np.zeros(r.shape)
        cl = np.zeros(r.shape)
        cd = np.zeros(r.shape)
        theta = np.zeros(r.shape)
        U_P = np.zeros(r.shape)
        U_T = np.zeros(r.shape)
        phi = np.zeros(r.shape)
        dQinduced = np.zeros(r.shape)
        dQprofile = np.zeros(r.shape)
        Qinduced = np.zeros(r.shape)
        Qprofile = np.zeros(r.shape)
        mach = np.zeros(r.shape)


        rearLiftProportion = 0
        advancingLiftProportion = 0
        tol = tolerancePct / 100
        liftDeficitPct = 9

        contours = ['dT', 'alphaEffective', 'mach', 'dL', 'dD', 'theta', 'dQinduced', 'dQprofile', 'phi']
        if self.animate:
            plt.ion()
            axes = []
            axes_cb = []


            figure = plt.figure(figsize=(14,12))
            for i in xrange(9):
                axes.append(plt.subplot(331+i, projection='polar'))
                var = vars()[contours[i]]
                if contours[i] in ['alphaEffective', 'theta', 'phi']:
                    var *= 180/math.pi
                c = plt.contourf(psi, r, var)
                axes_cb.append(plt.colorbar(c).ax)
                plt.title(contours[i])

            plt.draw()
        theta_0 = .1

        # this is the actual rotor trimming loop
        while np.isfinite(P) and steps<maxSteps and abs(theta_0)<math.pi/2 and not (abs(liftDeficitPct)<tol and abs(lastP-P)/P<tol and abs(rearLiftProportion-.5)/.5<tol and abs(advancingLiftProportion-advancingLiftBalance)/advancingLiftBalance<tol*300) and abs(P)<40000:
            steps += 1
            # find the effective blade section angle of attack
            beta = beta_0
            U_T = omega*r + V*sinpsi*np.cos(alpha_TPP) # local tangential velocity.
            U_P = inflow + V*np.sin(alpha_TPP) # local perpendicular velocity.
            theta = theta_0 + blade.twist + theta_1c*cospsi - theta_1s*sinpsi
            phi = np.arctan2(U_P, U_T)
            alphaEffective = theta - phi
            # funny modulo
            while np.any(alphaEffective > math.pi):
                alphaEffective[alphaEffective>math.pi] -= 2*math.pi
            while np.any(alphaEffective < -math.pi):
                alphaEffective[alphaEffective<-math.pi] += 2*math.pi
            # get the cl and cd data for blade sections.  Since we're using lookup tables, this is a quasi-steady solution.  
            # We may want to add in corrections for unstead aerodynamic effects and dynamic stall in the future.  
            # Dynamic stall is also important to find the vibration levels of the vehicle in high speed forward flight.
            cl = self.blade.cl(alphaEffective)
            cd = self.blade.cd(alphaEffective)
            # apply Prandtl-Glauert compressibility correction
            U_total = np.sqrt(U_T**2 + U_P**2) # Although we do not, we can assume U_T>>U_P, which is only untrue if U_T is small (i.e. on the retreating side in high-speed flight), in which case mach numbers will be small anyway, so should be ok.
            mach = U_total / speedOfSound
            cd[mach>blade.dragDivergenceMachNumber] = (mach[mach>blade.dragDivergenceMachNumber]-blade.dragDivergenceMachNumber)*.6 + cd[mach>blade.dragDivergenceMachNumber] # drag divergence
            mach[mach>=1] = .999999
            cl = cl / np.sqrt(1-mach**2)
            #cl[~np.isfinite(cl)] = 0. # if local mach>1, set cl=0.  This correction doesn't apply in those ranges, but this is probably acceptable and will allow trim solutions even with local M>1.  In combination with drag divergence, it should be an OK approximation.
            # Find the rotor sectional (1D) lift and drag
            dL = .5 * rho * U_total**2 * chord/R * cl
            dD = .5 * rho * U_total**2 * chord/R * cd
            # Calculate the piecewise (2D) lift and drag

            cosphi = np.cos(phi)
            sinphi = np.sin(phi)

            dT = (dL*cosphi - dD*sinphi) * numblades / (2*math.pi) # thrust per unit area
            dFx = (dD*cosphi +dL*sinphi) * numblades / (2*math.pi) # rotor plane force per unit area
            dPinduced = inflow*dT # induced power per unit area
            dDprofile = dD*cosphi * numblades / (2*math.pi) # per unit area
            # Integrate over the rotor surface
            lastT = T
            T = np.sum(dT * dr * r*dpsi) # Total Thrust
            # print T / diskArea
            #dQinduced = dDinduced * r * dr * r*dpsi
            dQprofile = dDprofile * r * dr * r*dpsi
            #Qinduced = np.sum(dDinduced * r * dr * r*dpsi)
            Qprofile = np.sum(dDprofile * r * dr * r*dpsi)
            #Pinduced = Qinduced * omega / 550 # np.sum(dDinduced * U_T) / 550
            Pinduced = np.sum(dPinduced * dr * r*dpsi) / 550
            Pprofile = Qprofile * omega / 550 # np.sum(dDprofile * U_T) / 550
            secondLastP = lastP
            lastP = P
            P = Pinduced + Pprofile
            # find how much lift is missing
            liftDeficitPct = (totalThrust - T) / abs(T)
            averageInflow = inflow.sum() / inflow.size

            # Calculate trim angles
            beta_0 = lockNumber / (2.*math.pi) * T / (.5*rho*math.pi*R**2*Vtip**2) # should replace this with a real calculation involving blade mass, but it's good enough for now
            # balance the rotor
            advancingLiftProportion = np.sum(np.ma.array(dT, mask=sinpsi<0)*dr*r*dpsi) / T # dT[sinpsi>0]) / T  # proportion of lift contributed by the advancing side
            rearLiftProportion = np.sum(np.ma.array(dT, mask=cospsi<0)*dr*r*dpsi) / T # dT[cospsi>0]) / T  # proportion of lift contributed by the rear side
            pitch = (.5 - rearLiftProportion) / 100
            roll = (advancingLiftProportion - advancingLiftBalance) / 100
            # cap the max magnitude of the changes
            if pitch>0:
                pitch = min(pitch, 0.01)
            else:
                pitch = max(pitch, -0.01)
            if roll>0:
                roll = min(roll, 0.01)
            else:
                roll = max(roll, -0.01)
            # apply the changes
            theta_1c += pitch
            theta_1s += roll

            # Find vertical lift and adjust collective
            L = T * math.cos(alpha_TPP)
            dtheta_0 = (totalThrust - T) * 0.000005
            # cap the max change at 0.1
            if dtheta_0 > 0:
                dtheta_0 = min(dtheta_0, 0.01)
            else:
                dtheta_0 = max(dtheta_0, -0.01)

            # Check if we're headed the wrong direction.  This is just to kill it if we can tell we're not going to be able to trim.
            CChanges.append(dtheta_0 > 0) # true if theta is going up
            TChanges.append(T-lastT > 0) # true if thrust is going up
            if len(CChanges) > 200:
                if (all(CChanges) and not any(TChanges)): # or (not any(CChanges) and all(TChanges)): # collective has been going up always and thrust has always been going down
                    if self.debug: print('Consistently wrong direction of change!')
                    P = float('nan') # this is the easiest way to break the loop...
                CChanges.pop(0)
                TChanges.pop(0)
            theta_0 += dtheta_0


            if self.animate:
                for i in xrange(9):
                    axes[i].cla()
                    var = vars()[contours[i]]
                    if contours[i] in ['alphaEffective', 'theta', 'phi']:
                        var *= 180/math.pi
                    c = axes[i].contourf(psi, r, var)
                    axes_cb[i].cla()
                    figure.colorbar(c, cax=axes_cb[i])
                    axes[i].set_title(contours[i])
                plt.draw()

            if self.debug:
                theta_0_hist.append(theta_0)
                b0_hist.append(beta_0)
                t1c_hist.append(theta_1c)
                t1s_hist.append(theta_1s)
                thrust_hist.append(T)
                roll_hist.append(roll)
                pitch_hist.append(pitch)
                miscA_hist.append(rearLiftProportion)
                miscB_hist.append(advancingLiftProportion)
                coll = theta_0 * 180/math.pi
                b0 = beta_0 * 180/math.pi
                t1s = theta_1s * 180/math.pi
                t1c = theta_1c * 180/math.pi
                total = math.sqrt(Fx**2 + Fz**2)
                dthet = dtheta_0 #* 1000
                roll = roll
                pitch = pitch
                pvar(locals(), ('steps', 'total', 'T', 'dthet', 'coll', 'b0', 't1c', 't1s', 'Pinduced', 'Pprofile', 'P', 'averageInflow'))
            if steps == False:  # suicide switch
                time.sleep(100)
                P = float('nan')
            if abs(secondLastP - P)<10e-8 and abs(liftDeficitPct)<tol:  # if we're just flip-flopping around the solution but right up next to it
                if self.debug: print('breaking')
                break



        self.inflow = inflow
        self.theta_0 = theta_0
        self.theta_1c = theta_1c
        self.theta_1s = theta_1s
        self.beta_0 = beta_0
        self.theta_0_hist = theta_0_hist
        self.b0_hist = b0_hist
        self.t1c_hist = t1c_hist
        self.t1s_hist = t1s_hist
        self.thrust_hist = thrust_hist
        self.roll_hist = roll_hist
        self.pitch_hist = pitch_hist
        self.miscA_hist = miscA_hist
        self.miscB_hist = miscB_hist
        
        if abs(liftDeficitPct)<tol and abs(lastP-P)/lastP<tol and abs(theta_0)<math.pi/6.0 and abs(P)<40000 and (abs(rearLiftProportion-.5)/.5<tol and abs(advancingLiftProportion-advancingLiftBalance)/advancingLiftBalance<tol*300):
          P_total = Pinduced + Pprofile
        elif abs(liftDeficitPct)>tol: # Failure to make enough lift
          P_total = -1
#          print theta_0*180./math.pi
          Pinduced = float('nan')
          Pprofile = float('nan')
        elif abs(lastP-P)/lastP>tol: # Failure to converge power
          P_total = -2
          Pinduced = float('nan')
          Pprofile = float('nan')
        elif abs(theta_0)>math.pi/6.0: # Too much collective required
          P_total = -3
          Pinduced = float('nan')
          Pprofile = float('nan')
        elif abs(P)>40000: # Power unrealistically high
          P_total = -4
          Pinduced = float('nan')
          Pprofile = float('nan')
        elif abs(rearLiftProportion-.5)/.5>tol: # Unable to balance rotor longitudinally
          P_total = -5
          Pinduced = float('nan')
          Pprofile = float('nan')
        elif abs(advancingLiftProportion-advancingLiftBalance)/advancingLiftBalance>tol*300: # Unable to balance rotor laterally
          P_total = -6
          Pinduced = float('nan')
          Pprofile = float('nan')
        else: # Some unknown issue
          P_total = -7
          Pinduced = float('nan')
          Pprofile = float('nan')




        if self.animate:
            plt.ioff()
        if self.plot:
            import matplotlib.pyplot as plt
            for i in xrange(9):
                plt.subplot(331+i, projection='polar')
                var = vars()[contours[i]]
                if contours[i] in ['alphaEffective', 'theta', 'phi']:
                    var *= 180/math.pi
                    c = plt.contourf(psi, r, var, np.arange(-20, 20))
                else:
                    c = plt.contourf(psi, r, var)
                plt.colorbar(c)
                plt.title(contours[i])
            #plt.show()

        # pvar(locals(), ('V', 'Fx', 'Fz', 'P_total'))
        self.power = P_total
#        print "Advancing Lift B % = ", abs(advancingLiftProportion-advancingLiftBalance)/advancingLiftBalance
#        print "Collective = ", abs(theta_0)*180/math.pi
        TrimData = [averageInflow, theta_0*180/math.pi, beta_0*180/math.pi, theta_1c*180/math.pi, theta_1s*180/math.pi]
        if returnAll:
            return (P_total, Pinduced, Pprofile, TrimData)
        else:
            return P_total #(Pinduced, Pprofile)






            # dF_x = - dL*sinphi*cospsi - dD*cosphi*cospsi
            # dF_y = - dL*sinphi*sinpsi - dD*cosphi*sinpsi
            # dF_z = - dL*cosphi + dD*sinphi
            # dQ = - dL*sinphi - dD*cosphi
            # # Integrate over the disk surface
            # dF_x[~np.isfinite(dF_x)] = 0
            # dF_y[~np.isfinite(dF_y)] = 0
            # dF_z[~np.isfinite(dF_z)] = 0
            # dQ[~np.isfinite(dQ)] = 0
            # F_x = np.sum(dF_x * dr * dpsi) * numblades
            # F_y = np.sum(dF_y * dr * dpsi) * numblades
            # F_z = np.sum(dF_z * dr * dpsi) * numblades
            # Q = np.sum(dQ * dr * r * dpsi) * numblades

    def plot(self):
        pass