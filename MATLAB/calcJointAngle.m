function Angle = calcJointAngle(J)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
J = J/4;
%maxP = [135 170 90 255 255]; %Degrees
%minP = [45 10 -90 0 0];
minE = [60 25 32 2 0];
maxE = [180 220 255 85 255];

for i =1:5
    %Check Max Range
    if(J(i) > maxE(i))
        J(i) = maxE(i);
    end

    %Check Min
    if(J(i) < minE(i))
        J(i) = minE(i);
    end
end

%Joint0 Calculation
for (Joint = 0:4)
    if(Joint == 0)
        x = [45 135];
        Y = [180 60];
        Angle(Joint+1) = interp1(Y,x,J(Joint+1));
    end

    %Joint1 Calculations
    if(Joint == 1)
        x = [10 45 90 135 170];
        Y = [25 60 120 180 220];
        Angle(Joint+1) = interp1(Y,x,J(Joint+1));
    end
    %Joint3 Calculations
    if(Joint == 2)
        x = [90 0 -90];
        Y = [32 144 255];
        Angle(Joint+1) = interp1(Y,x,J(Joint+1));
    end
    
    if(Joint == 3)
        x = [-21 -30.5 -45 19 31 43 50];
        Y = [20 10 2 60 70 80 85];
        Angle(Joint+1) = interp1(Y,x,J(Joint+1));
    end
        
	if(Joint == 4)
        x = [-90 90];
        Y = [0 255];
        Angle(Joint+1) = J(Joint+1);%interp1(Y,x,J(Joint+1));
    end
    
end

end

