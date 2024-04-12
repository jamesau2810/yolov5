import library
w,t = library.waypoint_file_read('./Trial002_waypoints.txt')
print(w)
print(t)
velocity_x = 3
velocity_y = 4
library.Helipad_land_speed_factor(velocity_x, velocity_y,3,[2,1,-1],[1,1,1],[1,0.5,0])
