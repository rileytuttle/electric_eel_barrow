include <BOSL2/std.scad>
include <BOSL2/rounding.scad>
include <BOSL2/screws.scad>
include <common-dims.scad>
use <wheel-motor-sim.scad>
use <rosetta-stone/boards.scad>

function bracket_top_z_dist() = bracket_square_height +bracket_triangle_height - 20;

function strut_width() = driven_side_bracket_dist()+brake_side_bracket_dist()-bracket_thickness+2*bracket_strut_aligment_pocket_depth;

function plate_width() = driven_side_bracket_dist() + brake_side_bracket_dist() + bracket_thickness+10;
function top_plate_size() = [plate_width(), plate_height, plate_thickness];
// function top_plate_offset() = [(driven_side_bracket_dist()+brake_side_bracket_dist())/2 -brake_side_bracket_dist(), top_plate_v_offset, plywood_thickness];

function axle_x_offset_caster(caster=false) = caster ? -bracket_width/2 +10+axle_diam/2 : 0;

module bracket_side(anchor=CENTER, spin=0, orient=UP, caster=false)
{
    anchor_list = [
        named_anchor("axle-hole", [axle_x_offset_caster(caster), -(bracket_top_z_dist()), bracket_thickness/2], orient=DOWN),
        named_anchor("mount-hole1", [-bracket_mount_spacing/2, 0, 0], orient=BACK),
        named_anchor("mount-hole2", [bracket_mount_spacing/2, 0, 0], orient=BACK),
        named_anchor("strut-mount-hole1", [-(bracket_width/2-strut_thickness/2), -(strut_mount_hole_dist_to_top), bracket_thickness/2 -bracket_strut_aligment_pocket_depth]),
        named_anchor("strut-mount-hole2", [(bracket_width/2-strut_thickness/2), -(strut_mount_hole_dist_to_top), bracket_thickness/2 -bracket_strut_aligment_pocket_depth]),
    ];
    bottom_triangle_path = [
        [-bracket_width/2, 0],
        [bracket_width/2, 0],
        [caster ? -bracket_width/2+20 : 20/2, -bracket_triangle_height],
        [caster ? -bracket_width/2 : -20/2, -bracket_triangle_height],
    ];
    attachable(spin=spin, orient=orient, anchor=anchor, anchors=anchor_list) {
        color_this("brown")
        diff("bolt-hole axle-hole nut-access strut-alignment-pocket strut-mount-holes")
        cube([bracket_width, bracket_square_height, bracket_thickness], anchor=BACK) {
            position(FRONT)
            // rounded_prism(trapezoid(h=bracket_triangle_height, w1=bracket_width, w2=20), height=bracket_thickness, anchor=FRONT, spin=180, joint_sides=[0, 0, 8, 8])
            color_this("brown") rounded_prism(bottom_triangle_path, height=bracket_thickness, joint_sides=[0, 0, 8, 8]);
            tag("axle-hole") {
                if (caster) {
                    position(FRONT)
                    translate([axle_x_offset_caster(caster), -(bracket_triangle_height-20), 0])
                    cyl(h=bracket_thickness+1, d=axle_diam+1, anchor=CENTER);
                } else {
                    position(FRONT)
                    translate([0, -(bracket_triangle_height-20), 0])
                    cyl(h=bracket_thickness+1, d=axle_diam+1);
                }
            }
            tag("bolt-hole") {
                for(i=[-1,1]) {
                    position(BACK)
                    translate([i*(bracket_mount_spacing/2), 1, 0])
                    screw_hole("1/4-20", head="none", thread="none", length=20, orient=BACK, anchor=TOP);
                }
            }
            tag("nut-access") {
                for(i=[-1,1]) {
                    position(BACK)
                    translate([i*(bracket_mount_spacing/2),-10,0])
                    nut_trap_side(20, "1/4-20", anchor=CENTER, orient=BACK, spin=90);
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
                    position(BACK+BOTTOM)
                    translate([i*(bracket_width/2-strut_thickness/2), -strut_mount_hole_dist_to_top, 0])
                    screw_hole("1/4-20", head="hex", thread="none", length=bracket_thickness+5, anchor=TOP, orient=DOWN);
                }
            }
            
        }
        children();
    }
}

module bracket_strut(anchor=CENTER, spin=0, orient=UP, chain_port=false, caster=false)
{
    anchor_list = [
        named_anchor("mount-hole1", [-strut_width()/2, strut_height/2-strut_mount_hole_dist_to_top, 0], orient=LEFT),
        named_anchor("mount-hole2", [strut_width()/2, strut_height/2-strut_mount_hole_dist_to_top, 0], orient=RIGHT),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, size=[strut_width(), strut_height, strut_thickness], anchors=anchor_list) {
        color_this("brown")
        diff("mount-holes wheel-cutout chain-port")
        cube([strut_width(), strut_height, strut_thickness], anchor=CENTER) {
            tag("wheel-cutout") {
                position(FRONT)
                translate([0, 30, 0])
                cyl(d=strut_width(), l=strut_thickness+5, anchor=BACK);
                if (caster)
                {
                    position(BACK)
                    translate([wheel_bracket_centers_offset(), -10, 0])
                    cyl(l=wheel_thickness+2, d=wheel_diam, rounding=20, anchor=BACK, orient=RIGHT);
                }
            }
            tag("mount-holes") {
                position(LEFT+BACK)
                translate([-1,-strut_mount_hole_dist_to_top,0])
                screw_hole("1/4-20", thread=true, length=15, anchor=TOP, orient=LEFT);
                // cyl(d=strut_mount_hole_diam, l=15+1, anchor=BOTTOM+BACK, orient=RIGHT);
                position(RIGHT+BACK)
                translate([1,-strut_mount_hole_dist_to_top,0])
                screw_hole("1/4-20", thread=true, length=15, anchor=TOP, orient=RIGHT);
                // cyl(d=strut_mount_hole_diam, l=15+1, anchor=BOTTOM+BACK, orient=LEFT);
            }
            if(chain_port) {
                tag("chain-port")
                {
                    position(RIGHT+FRONT)
                    translate([-7.5, -1, 0])
                    rounded_prism(square([10, 30]), height=strut_thickness+2, anchor=RIGHT+FRONT, joint_sides=[0, 0, 3, 3]);
                }
            }
        }
        children();
    }
}
module top_plate(anchor=CENTER, spin=0, orient=UP) {
    plate_shape_square_points = [
        each square([top_plate_size()[0], top_plate_size()[1]], center=true)
    ];
    plate_shape_triangle_points = [
        each move([0, -top_plate_size()[1]/2, 0], p=trapezoid(h=40, w1=top_plate_size()[0]-60, w2=0, anchor=BOTTOM)),
    ];
    plate_shape = [
        for (i=[1,2,3,0]) plate_shape_square_points[i],
        for (i=[0,2,1]) plate_shape_triangle_points[i],
        plate_shape_square_points[1],
    ];
    // stroke(plate_shape, endcap2="arrow2");
    // stroke(plate_shape_square_points, endcap2="arrow");
    // stroke(plate_shape_triangle_points, endcap2="arrow");
    
    // wheel_mount_hole_offset = [-top_plate_offset()[0]+(driven_side_bracket_dist() - brake_side_bracket_dist())/2, 0];
    wheel_mount_hole_offset = [0,0];
    wheel_mount_hole_locs = get_mount_hole_locs([brake_side_bracket_dist() + driven_side_bracket_dist(), bracket_mount_spacing], wheel_mount_hole_offset);
    echo(wheel_mount_hole_offset);
    echo(wheel_mount_hole_locs);
    anchor_list = [
        named_anchor("wheel-mount-holes-center", [wheel_mount_hole_offset[0], wheel_mount_hole_offset[1], -top_plate_size()[2]/2]),
    ];
    attachable(anchor=anchor, spin=spin,orient=orient, size=top_plate_size(), anchors=anchor_list) {
        color_this("brown") difference() {
            translate([0,0,-top_plate_size()[2]/2])
            offset_sweep(plate_shape, height=top_plate_size()[2]);
                for (i=[0:3]) {
                    position(TOP+BACK)
                    translate(wheel_mount_hole_locs[i])
                    translate([0, -top_plate_size()[1]/2, 1])
                    cyl(l=5+1, d=15, anchor=TOP);
                    position(TOP+BACK)
                    translate(wheel_mount_hole_locs[i])
                    translate([0,-top_plate_size()[1]/2,0.5])
                    cyl(l=top_plate_size()[2]+1, d=in_to_mm(0.25), anchor=TOP);
                }
        }
        children();
    }
}

module motor_top_plate(anchor=CENTER, spin=0, orient=UP, slotspread=10, motor_mount_center_offset=[0,0]) {
    motor_top_plate_height = 130;
    mount_center_y_offset = (motor_top_plate_height+top_plate_size()[0])/2 + motor_mount_center_offset[1] - slotspread/2;
    echo("y offset", mount_center_y_offset);
    top_plate_motor_mount_hole_locs = get_mount_hole_locs(motor_mount_hole_spacing, [motor_mount_center_offset[0], mount_center_y_offset]);
    plate_shape_square_points = [
        each square([top_plate_size()[0], motor_top_plate_height], center=true)
    ];
    plate_shape_triangle_points = [
        each move([0, motor_top_plate_height/2, 0], p=trapezoid(h=40, w1=top_plate_size()[0]-60, w2=0, anchor=BOTTOM)),
    ];
    plate_shape = [
        for (i=[3, 0, 1, 2]) plate_shape_square_points[i],
        for (i=[1,2,0]) plate_shape_triangle_points[i],
        plate_shape_square_points[3],
    ];
    anchor_list = [
        named_anchor("motor-mount-center", [motor_mount_center_offset[0], mount_center_y_offset, -top_plate_size()[2]/2]),
    ];
    // stroke(plate_shape, endcap2="arrow");
    attachable(anchor=anchor, spin=spin, orient=UP, size=[top_plate_size()[0], motor_top_plate_height, top_plate_size()[2]], anchors=anchor_list) {
        color_this("brown") difference() {
            translate([0, 0, -top_plate_size()[2]/2])
            offset_sweep(plate_shape, height=top_plate_size()[2]);
            for (i=[0:3]) {
                position(BOTTOM)
                translate(top_plate_motor_mount_hole_locs[i])
                slot(d=motor_mount_hole_diam, spread=slotspread, h=top_plate_size()[2]+0.01, anchor=BOTTOM, spin=90, round_radius=3);
            }
        }
        children();
    }
}

// bracket_side(caster=false);
// bracket_strut(caster=true, chain_port=true);

// motor_top_plate();
