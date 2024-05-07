function simIn = initAqua(Tfinal, ICstruct, orbitStruct, plantStruct, distStruct)

masterModel = 'aquaMasterModel';
load_system(masterModel)

masterWS = get_param(masterModel, 'modelworkspace');
masterWS.assignin('R0', ICstruct.R0)

masterWS.assignin('Tfinal', Tfinal)
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput(masterModel);
simIn.ExternalInput = M;

initOrbital(orbitStruct.orbitType, orbitStruct.dataSource)

initPlant(plantStruct.I_sim, plantStruct.axesFlag, plantStruct.dynamicsType, ...
            plantStruct.attitudeType, plantStruct.sequence, ICstruct)

initDisturbance(distStruct.disturbance, plantStruct)


% save_system(masterModel)