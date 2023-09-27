include <BOSL2/std.scad>
include <BOSL2/rounding.scad>

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

strut_thickness = 15;
strut_width = wheel_thickness+sprocket_ped_length+brake_ped_length+2*bracket_strut_aligment_pocket_depth+sprocket_thickness;
strut_height = 60;
strut_mount_hole_dist_to_top = 5;
strut_mount_hole_diam = 10;

module simulated_wheel_and_sprocket()
{
    anchor_list = [
        named_anchor("sprocket_top", [wheel_thickness / 2 + sprocket_ped_length + sprocket_thickness, 0, 0], orient=RIGHT),
        named_anchor("brake_ped_top", [-(wheel_thickness/2 + brake_ped_length), 0, 0], orient=LEFT),
    ];
    attachable(anchor=CENTER, orient=UP, spin=0, size=[wheel_diam, wheel_diam, axle_length], anchors=anchor_list) {
        // wheel
        rotate([0, 90, 0])
        cyl(h=wheel_thickness, d=wheel_diam, rounding1=20, rounding2=20) {
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
                cube([strut_thickness, strut_height, bracket_strut_aligment_pocket_depth+0.1], anchor=LEFT+BACK+TOP);
                position(BACK+TOP)
                translate([bracket_width/2,0,0.1])
                cube([strut_thickness, strut_height, bracket_strut_aligment_pocket_depth+0.1], anchor=RIGHT+BACK+TOP);
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

module simulated_motor(anchor=CENTER, spin=0, orient=UP) {
    attachable(anchor=anchor, spin=spin, orient=orient) {
        rotate([0, 90,0])
        diff("mount-holes")
        cyl(l=70, d=100) {
            position(TOP)
            cyl(l=30, d=10, anchor=BOTTOM);
            position(TOP)
            translate([0,0,20])
            cyl(l=3, d=20, anchor=BOTTOM);
            position(BOTTOM)
            cyl(l=5, d=30, anchor=BOTTOM, orient=DOWN);
            position(RIGHT)
            cube([55, 110, 3], anchor=BOTTOM, orient=RIGHT)
                tag("mount-holes") {
                    for (i=[-1, 1]) {
                        for (j=[-1, 1]) {
                            translate([i*20, j*45, 0])
                            cyl(l=3+1, d=4, anchor=CENTER);
                        }
                    }
                }
        }
        children();
    }
}

// simulated_wheel_and_sprocket() {
//     position("brake_ped_top")
//     bracket_side(anchor="axle-hole", orient=RIGHT, spin=90) {
//         position("strut-mount-hole1")
//         bracket_strut(anchor="mount-hole1", orient=LEFT);
//     }
//     position("sprocket_top")
//     bracket_side(anchor="axle-hole", orient=LEFT, spin=-90) {
//         position("strut-mount-hole1")
//         bracket_strut(anchor="mount-hole1", orient=LEFT);
//     }
// }

// // fake plywood
// translate([0, 0, bracket_square_height+bracket_triangle_height-20])
// #cube([200, 600, 5], anchor=BOTTOM);

simulated_motor();
