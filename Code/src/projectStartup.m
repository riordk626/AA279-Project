%% Project Startup File

projectRoot = mfilename('fullpath');
fileName = mfilename;
lenFileName = length(fileName);
projectRoot = projectRoot(1:end-lenFileName);
projectRoot = [projectRoot, '../..'];
addpath(projectRoot)

% Add paths
folders = {'/Code/src', '/Code/models'};

n = length(folders);

for i=1:n
    addpath([projectRoot, folders{i}])
end