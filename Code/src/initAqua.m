function simIn = initAqua(Tfinal, targetAttitude, ICstruct, orbitStruct,...
    plantStruct, distStruct, sensorStruct)

masterModel = 'aquaMasterModel';
load_system(masterModel)

masterWS = get_param(masterModel, 'modelworkspace');
masterWS.assignin('R0', ICstruct.R0)
% masterWS.assignin('A_ptob', plantStruct.A_ptob)

masterWS.assignin('Tfinal', Tfinal)

simIn = Simulink.SimulationInput(masterModel);
targetAttitude_data = targetAttitude;
targetAttitude_data(:,:,2) = targetAttitude;
R_RTNtoPdes = timeseries(targetAttitude_data, [0 Tfinal]);
simIn.ExternalInput = R_RTNtoPdes;

initOrbital(orbitStruct.orbitType, orbitStruct.dataSource)

initPlant(plantStruct.I_sim, plantStruct.axesFlag, plantStruct.dynamicsType, ...
            plantStruct.attitudeType, plantStruct.sequence, ICstruct)

initDisturbance(distStruct.disturbance, plantStruct, distStruct.dataSource)

initAttitudeSensor(sensorStruct.measProcess, sensorStruct.attitudeNoiseFactor,...
                    sensorStruct.attitudeSensorSolver, sensorStruct.starCatalog,...
                    sensorStruct.attitudeFileName)


% save_system(masterModel)