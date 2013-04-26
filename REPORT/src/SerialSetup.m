%% EECS567 Serial Setup
% Included for completeness.
COMPORT = 'COM5' %Differs on each PC.
delete(instrfind) %Deletes all instrument objects 
s = serial(COMPORT); %Creates the Serial Port Object
s.BytesAvailableFcnMode = 'terminator'; %Serial Stream of sends 5 Joints
% encoder counts followed by CR/LF
s.Terminator = 'CR/LF'
s.BytesAvailableFcn = @mycallback %Function Called when CR/LF is seen
%Setup Data Storage Locations (Can create others as well)
s.UserData.isNew = 0;
s.UserData.newData = 0;
 %Opens the Serial Communcation if currently not open 
 if strcmp(s.Status,'closed'), fopen(s); end