function [pos, tgps] = load_gps_meters(gps)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%Extract the lat, long, time and alt data from the gps table
lat = table2array(gps(:,3)); %radians
long = table2array(gps(:,4)); %radians
alt = table2array(gps(:,5)); %meters
tgps = table2array(gps(:,2)); %seconds
% lla = [lat long alt];
% % Get the Earth Centered, Earth fixed cartesian co-ordinates in meters
% p = lla2ecef(lla);
% % Get the x, y and z co-ordinates from the array
% xm = p(:,1);
% ym = p(:,2);
% zm = p(:,3);

% Convert from radians to degree.
lat_deg = rad2deg(lat);
long_deg = rad2deg(long);
%Using deg2utm
[xutm, yutm, zone] = deg2utm(lat_deg, long_deg);
zutm = alt;
pos = [xutm yutm zutm];

end

