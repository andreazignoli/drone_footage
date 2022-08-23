function [ d ] = distance_points(P1,P2)
%DISTANCE_POINTS calculates the distance between two points P1 and P2
%   d = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)

d = sqrt((P1(1)-P2(1))^2 + (P1(2)-P2(2))^2 + (P1(3)-P2(3))^2);

end

