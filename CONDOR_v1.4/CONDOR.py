# CONDOR
# Integrated Product Lifecylce Engineering Labratory

"""This the run file. All parameters for running are set in the input file. 
Set the associated variabels to either True or False in order to run that script. All
setup and running can be done from the associated Master_Input file without the 
need to go any further into the other codes."""

""" Running of selected Files"""
class CONDOR:
    def __init__(self, C):
        self.C = C
        C['BEMT Options']['BEMT_animate'] = False
        self.BEMT_Run = C['Code Selection']['BEMT_Run']
        self.rf_Run = C['Code Selection']['rf_Run']
        self.vehicle_Run = C['Code Selection']['vehicle_Run']
        # Till the BEMT_Run file is working it will be preset to not run
#        C['Code Selection']['BEMT_Run']=False
    def Run_Condor(self):
        if self.BEMT_Run:# Run BEMT Code
            from BEMT_Run import run_BEMT
            run_BEMT(self.C)
        if self.rf_Run:# Run rf
            from rf_Run import run_rf
            run_rf(self.C)
        if self.vehicle_Run:# Run vehicle
            from vehicle_Run import run_vehicle
            run_vehicle(self.C)

""" Code for Command Prompt Running """
if __name__ == '__main__':
    debug = True
    from configobj import ConfigObj
    from validate import Validator
    from sys import argv
    # Extract Input file from command input
    CDOR = argv[0]
    Config_File = argv[1]
    C = ConfigObj(Config_File, configspec = 'Config/Configspec/Master_Input.configspec')
    Cvdt = Validator()
    C.validate(Cvdt)
    # Run Class CONDOR
    blah = CONDOR(C)
    r = blah.Run_Condor()
