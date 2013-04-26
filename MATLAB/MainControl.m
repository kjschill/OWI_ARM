%ME567 PROJECT
%%CONFIGURATION
COMPORT = 'COM5' %Differs on each PC.
%% Setup Serial Communication to the OWI-565 Robotic ARM
delete(instrfind) %Deletes all instrument objects 
%as we know we only have one
s = serial(COMPORT);
s.BytesAvailableFcnMode = 'terminator'; %Serial Stream of sends 5 Joints
% Encoder counts followed by CR/LF
s.Terminator = 'CR/LF'
s.BytesAvailableFcn = @mycallback %Function Called wen CR/LF is seen
%Setup Data Storage Locations
s.UserData.isNew = 0
s.UserData.newData = 0;
 %Opens the Serial Communcation Port
 if strcmp(s.Status,'closed'), fopen(s); end
% get(s) %Uncomment this line and it gives all the COM port stats. Useful
% for debugging. 
% fread(s,1,'char') %Function to read the Serial Port

%Setup Route

%Loop
%   1. Calculate First Location and send to Robot
%   2. Calculate Next Location
%   3. When eror is below X Send New Point
%   4. Repeat 2 and 3 until path complete
%   5. Drink Beer

DesiredPoistion = [[90 90 0 -45 10]
                    [50 45 -45 -45 0]
                    [50 25 -55 -45 0]
                    [50 25 -55 -45 70]
                    [50 45 -45 -45 70]
                    [110 50 -45 -45 70]
                    [110 45 -55 -45 70]
                    [110 28 -55 -45 70]
                    [110 28 -55 -45 0]
                    [110 45 -55 -45 0]
                    [90 90 0 -45 0]];
its = 0;
for i = 1:size(DesiredPoistion,1)
    %Send New Command
    pause
    for j = 1:5
        setJointPos(s,j-1,DesiredPoistion(i,j))
        pause(.05)
    end
    while(max(s.UserData.newData(1:3)-DesiredPoistion(i,1:3))>3)
        its = its+1
        if(its>20)
            break
        end
        s.UserData.newData(1:3)-DesiredPoistion(i,1:3)
        pause(.25)
    end
    its = 0;
    
end
 
                
% %Setup Global
% global RUNNING;
% RUNNING=1;
% 
% 
% %Setup for Live Plotting
% %  x-axis in the data plot.
% plotData=zeros(1,6);
% newData=[];
% pfig = figure;
% 
% while RUNNING
%     
%     % wait until we have new data
%     if s.UserData.isNew==1        
%         % get the data from serial port object (data will be row-oriented)    
%         newData=s.UserData.newData';
%         
%         % indicate that data has been read
%         s.UserData.isNew=0;
%         
%         % concatenate new data for plotting
%         plotData=[plotData; newData];
%         
%         % plot the data
%         plot(pfig,plotData);
%         
%         drawnow;
%     end
%     
%     % The loop will exit when the user presses return, using the
%     %  KeyPressFcn of the plot window    
% end

setJointPos(s,1,120)
pause
fclose(s)
delete(s)
clear(s)
fscanf(s,'%i',30)

s.fclose
get(s)
clc
fread(s, s.BytesAvailable)
