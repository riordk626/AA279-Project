function simIn = initAqua(Tfinal, extInputStruct, ICstruct, orbitStruct, plantStruct, distStruct)

masterModel = 'aquaMasterModel';
load_system(masterModel)

masterWS = get_param(masterModel, 'modelworkspace');
masterWS.assignin('R0', ICstruct.R0)

masterWS.assignin('Tfinal', Tfinal)

simIn = Simulink.SimulationInput(masterModel);
extInputFields = fieldnames(extInputStruct);
inDS = createInputDataset(masterModel);
for i = length(extInputFields)
    name = extInputFields{i};
    var = extInputStruct.(name);
    inDS{i} = var;
end
simIn = setExternalInput(simIn, );

initOrbital(orbitStruct.orbitType, orbitStruct.dataSource)

initPlant(plantStruct.I_sim, plantStruct.axesFlag, plantStruct.dynamicsType, ...
            plantStruct.attitudeType, plantStruct.sequence, ICstruct)

initDisturbance(distStruct.disturbance, plantStruct, distStruct.dataSource)


% save_system(masterModel)