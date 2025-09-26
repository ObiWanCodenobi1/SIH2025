% Install IRSL library (package: +irsl)

thisDir = fileparts(mfilename('/Users/UserName/Desktop/IRSL'));  % IRSL folder and change path 
addpath(thisDir);                             % add IRSL to path
savepath;                                     % optional, make permanent

fprintf('✅ IRSL library installed. Use functions as irsl.FunctionName()\n');
