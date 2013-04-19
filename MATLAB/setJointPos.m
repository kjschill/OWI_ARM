function setJointPos(Obj,Joint,Position)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%Bounds Checking
if(Joint>4)
    return
end

maxP = [135 170 90 45 255]; %Degrees
minP = [45 10 -90 -45 0];

%Check Max Range
if(Position > maxP(Joint+1))
    Position = maxP(Joint+1);
end

%Check Min
if(Position<minP(Joint+1))
    Position = minP(Joint+1);
end

%Joint0 Calculation
if(Joint == 0)
    x = [45 135];
    Y = [180 60];
    Position = interp1(x,Y,Position);
end

%Joint1 Calculations
if(Joint == 1)
    x = [10 45 90 135 170];
    Y = [25 60 120 180 220];
    Position = interp1(x,Y,Position);
end
%Joint3 Calculations
if(Joint == 2)
    x = [90 0 -90];
    Y = [32 144 255];
    Position = interp1(x,Y,Position);
end

%Joint3 Calculations
if(Joint == 3)
    x = [-21 -30.5 -45 19 31 43 50];
    Y = [20 10 2 60 70 80 85];
    Position = interp1(x,Y,Position);
end

if(Position>255 || Position<0)
    return
end

CMD = [85 170 0 0];
CMD(3) = Joint;
CMD(4) = Position;
fwrite(Obj,CMD);
end

