include <BOSL2/std.scad>
include <BOSL2/rounding.scad>
include <rosetta-stone/std.scad>

// wheel dims
wheel_diam = 180;
axle_length = 165;
axle_diam = 13;
wheel_thickness = 50;
sprocket_diam = 113;
sprocket_thickness = 3;
sprocket_ped_length = 20;
sprocket_ped_diam = 45;
brake_ped_length = 25;
brake_ped_diam = 37;

// bracket dims
bracket_width = 150;
bracket_thickness = 20;
bracket_square_height = 60;
bracket_triangle_height = 60;
bracket_mount_spacing = 80;
bracket_mount_nut_access_width = 20;
bracket_mount_nut_access_height = 10;
bracket_strut_aligment_pocket_depth = 3;
chain_space = 8;

// get the dist from center to driven side bracket center
function driven_side_bracket_dist() = wheel_thickness/2+sprocket_ped_length+chain_space+sprocket_thickness + bracket_thickness / 2;
// get the dist from center to brake side bracket center
function brake_side_bracket_dist() = wheel_thickness/2+brake_ped_length+bracket_thickness/2;

strut_thickness = 15;
strut_width = driven_side_bracket_dist()+brake_side_bracket_dist()-bracket_thickness+2*bracket_strut_aligment_pocket_depth;
strut_height = 60;
strut_mount_hole_dist_to_top = 5;
strut_mount_hole_diam = 10;



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
            position(TOP)
            cyl(d=sprocket_diam, h=sprocket_thickness, anchor=BOTTOM);
            
            // brake pedastal
            position(BOTTOM)
            orient(DOWN)
            cyl(h=brake_ped_length, d=brake_ped_diam, anchor=BOTTOM);
            // axle
            position(CENTER)
            cyl(h=axle_length, d=axle_diam);
        }
        children();
    }
}

module bracket_side(anchor=CENTER, spin=0, orient=UP)
{
    anchor_list = [
        named_anchor("axle-hole", [0, -(bracket_square_height + bracket_triangle_height - 20), bracket_thickness/2]),
        named_anchor("mount-hole1", [-bracket_mount_spacing/2, 0, 0], orient=BACK),
        named_anchor("mount-hole2", [bracket_mount_spacing/2, 0, 0], orient=BACK),
        named_anchor("strut-mount-hole1", [-(bracket_width/2-strut_thickness/2), -(strut_mount_hole_dist_to_top+strut_mount_hole_diam/2), bracket_thickness/2 -bracket_strut_aligment_pocket_depth]),
        named_anchor("strut-mount-hole2", [(bracket_width/2-strut_thickness/2), -(strut_mount_hole_dist_to_top+strut_mount_hole_diam/2), bracket_thickness/2 -bracket_strut_aligment_pocket_depth]),
    ];
    attachable(spin=spin, orient=orient, anchor=anchor, anchors=anchor_list) {
        diff("bolt-hole axle-hole nut-access strut-alignment-pocket strut-mount-holes")
        cube([bracket_width, bracket_square_height, bracket_thickness], anchor=BACK) {
            position(FRONT)
            rounded_prism(trapezoid(h=bracket_triangle_height, w1=bracket_width, w2=20), height=bracket_thickness, anchor=FRONT, spin=180, joint_sides=[0, 0, 8, 8])
                tag("axle-hole") {
                    position(BACK)
                    translate([0, -20, 0])
                    cyl(h=axle_length, d=axle_diam+1);
                }
            tag("bolt-hole") {
                for(i=[-1,1]) {
                    position(BACK)
                    translate([i*(bracket_mount_spacing/2), +1, 0])
                    cyl(d=10, l=20, anchor=BOTTOM, orient=FRONT);
                }
            }
            tag("nut-access") {
                for(i=[-1,1]) {
                    position(BACK)
                    translate([i*(bracket_mount_spacing/2),-10,0])
                    cube([bracket_mount_nut_access_width, bracket_mount_nut_access_height, bracket_thickness+5], anchor=CENTER+BACK);
                }
            }
            tag("strut-alignment-pocket") {
                position(BACK+TOP)
                translate([-bracket_width/2,0,0.1])
                cube([strut_thickness+1, strut_height+1, bracket_strut_aligment_pocket_depth+0.1], anchor=LEFT+BACK+TOP);
                position(BACK+TOP)
                translate([bracket_width/2,0,0.1])
                cube([strut_thickness+1, strut_height+1, bracket_strut_aligment_pocket_depth+0.1], anchor=RIGHT+BACK+TOP);
            }
            tag("strut-mount-holes") {
                for (i=[-1, 1]) {
                    position(BACK)
                    translate([i*(bracket_width/2-strut_thickness/2), -strut_mount_hole_dist_to_top, 0])
                    cyl(d=strut_mount_hole_diam, l=bracket_thickness+5, anchor = CENTER+BACK);
                }
            }
            
        }
        children();
    }
}

// module bracket_top(anchor=CENTER, spin=0, orient=UP)
// {
//     cube([30, bracket_width, bracket_thickness]);
// }

module bracket_strut(anchor=CENTER, spin=0, orient=UP)
{
    anchor_list = [
        named_anchor("mount-hole1", [-strut_width/2, strut_height/2-strut_mount_hole_dist_to_top-strut_mount_hole_diam/2, 0], orient=LEFT),
        named_anchor("mount-hole2", [strut_width/2, strut_height/2-strut_mount_hole_dist_to_top-strut_mount_hole_diam/2, 0], orient=RIGHT),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, size=[strut_width, strut_height, strut_thickness], anchors=anchor_list) {
        diff("mount-holes wheel-cutout")
        cube([strut_width, strut_height, strut_thickness], anchor=CENTER) {
            tag("wheel-cutout") {
                position(FRONT)
                translate([0, 30, 0])
                cyl(d=strut_width, l=strut_thickness+5, anchor=BACK);
            }
            tag("mount-holes") {
                position(LEFT+BACK)
                translate([-1,-strut_mount_hole_dist_to_top,0])
                cyl(d=strut_mount_hole_diam, l=15+1, anchor=BOTTOM+BACK, orient=RIGHT);
                position(RIGHT+BACK)
                translate([1,-strut_mount_hole_dist_to_top,0])
                cyl(d=strut_mount_hole_diam, l=15+1, anchor=BOTTOM+BACK, orient=LEFT);
            }
        }
        children();
    }
}

motor_width = 70;
motor_diam = 100;
motor_mount_plate_thickness = 3;
motor_axle_length = 30;
motor_sprocket_dist = 20;
motor_sprocket_thickness = 3;
motor_axle_diam = 10;
motor_sprocket_diam = 20;
motor_mount_plate_width = 55;
motor_mount_plate_height = 110;
motor_mount_plate_hole_h_space = 40;
motor_mount_plate_hole_v_space = 90;

module simulated_motor(anchor=CENTER, spin=0, orient=UP) {
    anchor_list = [
        named_anchor("mount-plate", [0, 0, -(motor_diam/2 + motor_mount_plate_thickness)], orient=DOWN),
        named_anchor("sprocket", [motor_diam/2+motor_sprocket_dist+motor_sprocket_thickness/2, 0, 0], orient=RIGHT),
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
            position(TOP)
            translate([0,0,motor_sprocket_dist])
            cyl(l=motor_sprocket_thickness, d=motor_sprocket_diam, anchor=BOTTOM);
            // bump
            position(BOTTOM)
            cyl(l=5, d=30, anchor=BOTTOM, orient=DOWN);
            // mount plate
            position(RIGHT)
            cube([motor_mount_plate_width, motor_mount_plate_height, motor_mount_plate_thickness], anchor=BOTTOM, orient=RIGHT)
                tag("mount-holes") {
                    for (i=[-(motor_mount_plate_hole_h_space/2), motor_mount_plate_hole_h_space/2]) {
                        for (j=[-(motor_mount_plate_hole_v_space/2), (motor_mount_plate_hole_v_space/2)]) {
                            translate([i, j, 0])
                            cyl(l=motor_mount_plate_thickness+1, d=4, anchor=CENTER);
                        }
                    }
                }
        }
        children();
    }
}
sprocket_alignment = (motor_width/2 + motor_sprocket_dist+motor_sprocket_thickness/2) - (wheel_thickness/2+sprocket_ped_length+sprocket_thickness/2);
motor_mount_v_dist = 140;

module wheel_and_bracket(anchor=CENTER, spin=0, orient=UP) {
    anchor_list = [
        named_anchor("driven-sprocket",[wheel_thickness/2+sprocket_ped_length+sprocket_thickness/2, 0, 0], orient=RIGHT),
        // named_anchor("bracket-top", [0, 0, bracket_square_height+bracket_triangle_height-20], orient=UP),
        named_anchor("bracket-top", [0, 0, bracket_square_height+bracket_triangle_height-20], orient=UP),
        named_anchor("mount-plate", [-sprocket_alignment, -motor_mount_v_dist, bracket_square_height+bracket_triangle_height-20], orient=UP),
        named_anchor("driven-side-bracket-top", [driven_side_bracket_dist(), 0, bracket_square_height+bracket_triangle_height-20], orient=UP),
        named_anchor("brake-side-bracket-top", [-brake_side_bracket_dist(), 0, bracket_square_height+bracket_triangle_height-20], orient=UP),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, anchors=anchor_list) {
        simulated_wheel_and_sprocket() {
            position("brake_ped_top")
            bracket_side(anchor="axle-hole", orient=RIGHT, spin=90) {
                position("strut-mount-hole1")
                bracket_strut(anchor="mount-hole1", orient=LEFT);
            }
            position("sprocket_top")
            translate([chain_space, 0, 0])
            bracket_side(anchor="axle-hole", orient=LEFT, spin=-90) {
                position("strut-mount-hole1")
                bracket_strut(anchor="mount-hole1", orient=LEFT);
            }
        }
        children();
    }
}

plate_width = driven_side_bracket_dist() + brake_side_bracket_dist() + bracket_thickness+10;
plate_height = 300;
plate_thickness = 20;
module top_plate(anchor=CENTER, spin=0, orient=UP) {
    attachable(anchor=anchor, spin=spin,orient=orient, size=[plate_width, plate_height, plate_thickness]) {
        cuboid([plate_width, plate_height, plate_thickness], anchor=CENTER, rounding=5, teardrop=true );
        children();
    }
}

plywood_thickness = 5;

diff("motor-mount-slots wheel-mount-holes") {
wheel_and_bracket() {
    translate([-sprocket_alignment, -motor_mount_v_dist, 0])
    position("bracket-top")
    simulated_motor(anchor="mount-plate", orient=DOWN, spin=180);
    translate([(driven_side_bracket_dist()+brake_side_bracket_dist())/2 -brake_side_bracket_dist(), -60, plywood_thickness])
    position("bracket-top")
    color_this("brown")top_plate(BOTTOM);
    tag("motor-mount-slots") {
        for (i=[-motor_mount_plate_hole_h_space/2, motor_mount_plate_hole_h_space/2]) {
            for (j=[-motor_mount_plate_hole_v_space/2, motor_mount_plate_hole_v_space/2]) {
                position("mount-plate")
                translate([i,j,plywood_thickness+plate_thickness/2])
                slot(d=7, spread=10, h=plate_thickness+0.001, anchor=CENTER, spin=90, round_radius=3);
            }
        }
    }
    tag("wheel-mount-holes") {
        for (i=[-bracket_mount_spacing/2, bracket_mount_spacing/2]) {
            position("driven-side-bracket-top")
            translate([0,i,plywood_thickness+plate_thickness/2])
            cyl(l=plate_thickness+1, d=10,anchor=CENTER);
        }
        for (i=[-bracket_mount_spacing/2, bracket_mount_spacing/2]) {
            position("brake-side-bracket-top")
            translate([0,i,plywood_thickness+plate_thickness/2])
            cyl(l=plate_thickness+1, d=10,anchor=CENTER);
        }
    }
}
}
    

// // fake plywood
// translate([0, 0, bracket_square_height+bracket_triangle_height-20])
// color_this("orange") cube([200, 600, plywood_thickness], anchor=BOTTOM);
