function [quat_imu_earth] = quaternionFromAccMag(acc, mag)
%QUATERNIONFROMACCMAG Orientation quaternion from gravity and magnetic
%field vector.
    z_earth_imu =  acc/norm(acc);
    x_earth_imu = -cross(z_earth_imu,mag);
    x_earth_imu = x_earth_imu/norm(x_earth_imu);
    y_earth_imu = cross(z_earth_imu,x_earth_imu);
	quat_imu_earth = quaternionFromRotmat([x_earth_imu' y_earth_imu' z_earth_imu']');
end

