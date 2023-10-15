use <rosetta-stone/std.scad>

wheel_diam = 180;
axle_length = 165;
axle_diam = 13;
wheel_thickness = 50;
sprocket_diam = 113.5;
sprocket_thickness = 3;
sprocket_ped_length = 20;
sprocket_ped_diam = 45;
brake_ped_length = 25;
brake_ped_diam = 37;

motor_width = 70;
motor_diam = 100;
motor_axle_length = 30;
motor_sprocket_dist = 20;
motor_sprocket_thickness = 3;
motor_axle_diam = 10;
motor_sprocket_diam = 20;
motor_mount_plate_size = [55, 110, 3];
motor_mount_hole_spacing = [40, 90];
motor_mount_hole_diam = 6.5;
chain_space = 8;

bracket_width = 150;
bracket_thickness = 20;
bracket_square_height = 60;
bracket_triangle_height = 60;
bracket_mount_spacing = 80;
bracket_mount_nut_access_width = 20;
bracket_mount_nut_access_height = 10;
bracket_strut_aligment_pocket_depth = 3;

strut_thickness = 15;
strut_height = 60;
strut_mount_hole_dist_to_top = 10;
strut_mount_hole_diam = 10;

plate_height = 150;
plate_thickness = 10;
plywood_thickness = in_to_mm(0.75);
top_plate_v_offset = -60;

// caster dims
bearing_fill_plug_screw_name = "M7-1";
bearing_diam = 6;
bearing_fill_plug_screw_flat_height = 4;
channel_bearing_diam = 6.25;
channel_radius = 50;
n_bearings = 49;

// want this to fit through front door
cart_width = 30 * INCH;
cart_length = 42 * INCH;
cart_bottom_size = [cart_width, cart_length, plywood_thickness];
cart_side_size = [18 * INCH, cart_length, plywood_thickness];
