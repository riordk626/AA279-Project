function initAqua(Tfinal, ICstruct, orbitStruct, plantStruct, distStruct)

masterModel = 'aquaMasterModel';
load_system(masterModel)

masterWS = get_param(masterModel, 'modelworkspace');
masterWS.evalin('R0', ICstruct.R0)

masterWS.evalin('Tfinal', Tfinal)
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput(masterModel);
simIn.ExternalInput = M;

initOrbital(orbitStruct.orbitType)

initPlant(plantStruct.I_sim, plantStruct.axesFlags, plantStruct.dynamicesType, ...
            plantStruct.sequence, ICstruct)

initDisturbance(distStruct.disturbance)