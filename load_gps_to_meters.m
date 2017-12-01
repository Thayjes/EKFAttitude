%Load the gps data from the mat file (all the tables are loaded)
load('F:\Visnav Flight Data\20170719_Upper_Flight1\all.mat');
%Extract the lat, long, time and alt data from the gps table
lat = table2array(gps(:,3)); %radians
long = table2array(gps(:,4)); %radians
alt = table2array(gps(:,5)); %meters
t = table2array(gps(:,2)); %seconds
lla = [lat long alt];
% Get the Earth Centered, Earth fixed cartesian co-ordinates in meters
p = lla2ecef(lla);
% Get the x, y and z co-ordinates from the array
xm = p(:,1);
ym = p(:,2);
zm = p(:,3);