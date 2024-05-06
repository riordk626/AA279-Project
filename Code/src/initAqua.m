function initAqua(Tfinal, ICstruct, orbitStruct, plantStruct, distStruct)

masterModel = 'aquaMasterModel';
load_system(masterModel)

masterWS = get_param(masterModel, 'modelworkspace');

masterWS.evalin('Tfinal', Tfinal)
M = timeseries(zeros([3 2]), [0 Tfinal]);
simIn = Simulink.SimulationInput(masterModel);
simIn.ExternalInput = M;