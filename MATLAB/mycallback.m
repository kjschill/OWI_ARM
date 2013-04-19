function mycallback( Obj,event )
%MYCALLBACK Summary of this function goes here
%   Detailed explanation goes here
    %J = fscanf(Obj,'%i',30)
    % obj.UserData.newData = fscanf(Obj,'%i',30)
    
    %READ NEW DATA
  J = fscanf(Obj,'%i',30);
  size(J,1);
  %Make Sure it is the correct size
  if (size(J,1) == 5)
      J = calcJointAngle(J);
      Obj.UserData.newData=J;
  end
  %disp(J);
%    %J=fscanf(s,'%i',30)
%    if Obj.UserData.isNew==0
%         % indicate that we have new data
%         Obj.UserData.isNew = 1; 
%         Obj.UserData.newData = J;
%     else
%         % If the main loop has not had a chance to process the previous batch
%         % of data, then append this new data to the previous "new" data
%         Obj.UserData.newData=J%[Obj.UserData.newData Dnew];
%     end
end

