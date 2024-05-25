function initAttitudeSensor(measProcess, noiseFactor, attitudeSensorSolver, starCatalog, sourceFileName)
model = "attitudeSensors";
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.DataSource = 'MAT-File';
mws.FileName = sourceFileName;
mws.reload

mws.assignin('measProcess', measProcess)
mws.assignin('attitudeSensorSolver', attitudeSensorSolver)
mws.assignin('starCatalog', starCatalog)
mws.assignin('attitudeNoiseFactor', noiseFactor)

load("magConstants.mat", "B0", "Re", "mhat_ecef")
mws.assignin('B0', B0)
mws.assignin('Re', Re)
mws.assignin('mhat_ecef', mhat_ecef)