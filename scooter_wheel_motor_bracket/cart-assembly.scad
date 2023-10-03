use <bracket.scad>
include <rosetta-stone/std.scad>

plywood_thickness = in_to_mm(0.75);

bottom_plywood_size = [900,1200,plywood_thickness];

//plywood to mount everything to
color_this(hsv(39, 0.99, 0.8)) cube(bottom_plywood_size, anchor=CENTER)
for (i=[-350,350]) {
    position(BOTTOM)
    translate([i,450,0])
    wheel_bracket_motor_top_plate_assembly(top_plate_z_offset=plywood_thickness, anchor="bracket-top");
}

// for (i=[-400, 400])
// wheel_bracket_and_motor_assembly();

