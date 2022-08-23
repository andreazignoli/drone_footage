%% GPS2XYZ
%Transforms the longitude, latitude and altitude arrays into cartesian
%coordinates.
%Earth is approximated to a sphere
%Origin is the mean value of longitude and latitude on the plane tangent to
%the sphere
%X axis points to North, Y axis to West and Z axis point north
%function also returns the origin and the reference frame on the tangent
%plane.
%
%The datum used for GPS positioning is called WGS84 (World Geodetic System
%1984). It consists of a three-dimensional Cartesian coordinate system and
%an associated ellipsoid so that WGS84 positions can be described as either
%XYZ Cartesian coordinates or latitude, longitude and ellipsoid height
%coordinates. The origin of the datum is the Geocentre (the centre of mass
%of the Earth) and it is designed for positioning anywhere on Earth.
%The shape and size of the WGS84 biaxial ellipsoid is defined by the semi-major axis length
%a = 6378137.0 metres, and the reciprocal of flattening 1/ f = 298.257223563. 
%This ellipsoid is the same shape and size as the GRS80 ellipsoid.

%NOTA: per ora implementato come una sfera
function [XYZ, origin, nord, ovest, n] = GPS2XYZ(longitude,latitude,altitude)
  
  R0  = 6378388 ; % raggio della terra
  %conversion to degree
  lat = latitude  ./ 180 * pi; 
  lon = longitude ./ 180 * pi ;
  
  %Origin calculation
  % AZ modified
  % lon_ave = mean(lon);
  % lat_ave = mean(lat);
  % alt_ave = mean(altitude);
  lon_ave = lon(1);
  lat_ave = lat(1);
  alt_ave = altitude(1);
  
  %Normal vector
  n = [ cos(lat_ave)*cos(lon_ave), ...
        cos(lat_ave)*sin(lon_ave), ...
        sin(lat_ave) ] ;
  
  %origin position
  origin = (R0+alt_ave)*n;
  
  % X, Y unit vectors on tangent plane
  ovest = cross(n,[0 0 1]);
  ovest = ovest /norm(ovest); 

  nord  = cross(ovest,n);
  
  % Equivalent calculation
  %ovest = [sin(lon),-cos(lon),0];
  %nord  = [-cos(lon)*sin(lat), -sin(lon)*sin(lat),cos(lat)];
  
  
  % Coordinate of point P in ERCT
  PX = [ cos(lat).*cos(lon)] .* (R0+altitude) ;
  PY = [ cos(lat).*sin(lon)] .* (R0+altitude) ;
  PZ = [ sin(lat) ] .* (R0+altitude) ;
  
  P = [PX,PY,PZ];
  
  %Projection on tangent plane
  PO = (P-ones(size(lon))*origin);
  XYZ = [ PO * nord.', PO * ovest.', PO * n.'] ;
  
end

