include <BOSL2/std.scad>
include <common-dims.scad>
use <rosetta-stone/boards.scad>

// get the dist from center to driven side bracket center
function driven_side_bracket_dist() = wheel_thickness/2+sprocket_ped_length+chain_space+sprocket_thickness + bracket_thickness / 2;
// get the dist from center to brake side bracket center
function brake_side_bracket_dist() = wheel_thickness/2+brake_ped_length+bracket_thickness/2;
function wheel_bracket_centers_offset() = ((brake_ped_length+wheel_thickness/2) - (wheel_thickness/2 + sprocket_ped_length+sprocket_thickness+chain_space))/2;
module simulated_wheel_and_sprocket()
{
    anchor_list = [
        named_anchor("sprocket_top", [wheel_thickness / 2 + sprocket_ped_length + sprocket_thickness, 0, 0], orient=RIGHT),
        named_anchor("sprocket-center", [wheel_thickness / 2 + sprocket_ped_length + sprocket_thickness/2, 0, 0], orient=RIGHT),
        named_anchor("brake_ped_top", [-(wheel_thickness/2 + brake_ped_length), 0, 0], orient=LEFT),
    ];
    attachable(anchor=CENTER, orient=UP, spin=0, size=[wheel_diam, wheel_diam, axle_length], anchors=anchor_list) {
        // wheel
        rotate([0, 90, 0])
        color_this("grey") cyl(h=wheel_thickness, d=wheel_diam, rounding1=20, rounding2=20) {
            // sprocket pedastal
            position(TOP)
            cyl(h=sprocket_ped_length, d=sprocket_ped_diam, anchor=BOTTOM)
            // sprocket
            color_this("red") position(TOP)
            cyl(d=sprocket_diam, h=sprocket_thickness, anchor=BOTTOM);
            
            // brake pedastal
            position(BOTTOM)
            orient(DOWN)
            cyl(h=brake_ped_length, d=brake_ped_diam, anchor=BOTTOM);
            // axle
            color_this("grey") position(CENTER)
            cyl(h=axle_length, d=axle_diam);
        }
        children();
    }
}

module simulated_motor(anchor=CENTER, spin=0, orient=UP) {
    motor_mount_hole_locs = get_mount_hole_locs(motor_mount_hole_spacing);
    anchor_list = [
        named_anchor("mount-plate", [0, 0, -(motor_diam/2 + motor_mount_plate_size[2])], orient=DOWN),
        named_anchor("sprocket", [motor_width/2+motor_sprocket_dist+motor_sprocket_thickness/2, 0, 0], orient=RIGHT),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, size=[motor_width, motor_diam, motor_diam], anchors=anchor_list) {
        rotate([0, 90,0])
        diff("mount-holes")
        // main cylinder
        cyl(l=motor_width, d=motor_diam) {
            // axle
            position(TOP)
            cyl(l=motor_axle_length, d=motor_axle_diam, anchor=BOTTOM);
            // sprocket
            color_this("red") position(TOP)
            translate([0,0,motor_sprocket_dist])
            cyl(l=motor_sprocket_thickness, d=motor_sprocket_diam, anchor=BOTTOM);
            // bump
            position(BOTTOM)
            cyl(l=5, d=30, anchor=BOTTOM, orient=DOWN);
            // mount plate
            position(RIGHT)
            cube(motor_mount_plate_size, anchor=BOTTOM, orient=RIGHT)
                tag("mount-holes") {
                    for (i=[0:3]) {
                        translate(motor_mount_hole_locs[i])
                        cyl(l=motor_mount_plate_size[2]+1, d=motor_mount_hole_diam, anchor=CENTER);
                    }
                }
        }
        children();
    }
}

module simulated_chain(anchor=CENTER,orient=UP,spin=0, sprocket_to_sprocket_dist=200) {
    anchor_list = [
        named_anchor("sprocket-center", [0, 0, 0]),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, anchors=anchor_list) {
        difference() {
            hull() {
                cyl(d=motor_sprocket_diam+10,l=motor_sprocket_thickness+1);
                translate([sprocket_to_sprocket_dist, 0, 0])
                cyl(d=sprocket_diam+10, l=sprocket_thickness+1);
            }
            hull() {
                cyl(d=motor_sprocket_diam - 10, l=motor_sprocket_thickness+5);
                translate([sprocket_to_sprocket_dist, 0, 0])
                cyl(d=sprocket_diam - 10, l=sprocket_thickness+5);
            }
        }
        children();
    }
}
