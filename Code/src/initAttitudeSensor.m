function initAttitudeSensor(measProcess, attitudeSensorSolver, starCatalog, sourceFileName)
model = "attitudeSensors";
load_system(model)

mws = get_param(model, 'modelworkspace');
mws.DataSource = 'MAT-File';
mws.FileName = sourceFileName;
mws.reload

mws.assignin('measProcess', measProcess)
mws.assignin('attitudeSensorSolver', attitudeSensorSolver)
mws.assignin('starCatalog', starCatalog)
