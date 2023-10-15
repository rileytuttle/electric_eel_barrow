include <BOSL2/std.scad>
include <common-dims.scad>
use <wheel-motor-sim.scad>
use <brackets.scad>

motor_mount_v_dist = 140;

function sprocket_alignment() = (motor_width/2 + motor_sprocket_dist+motor_sprocket_thickness/2) - (wheel_thickness/2+sprocket_ped_length+sprocket_thickness/2);

function chain_angle() = atan((bracket_top_z_dist()-(motor_diam / 2 + motor_mount_plate_size[2]))/motor_mount_v_dist) + 90;

function sprocket_to_sprocket_dist() = ((bracket_top_z_dist()-(motor_diam/2+motor_mount_plate_size[2]))^2 + motor_mount_v_dist^2)^0.5;

// motor mount offset relative to the wheel center
function motor_mount_center_offset() = [-sprocket_alignment(), -motor_mount_v_dist];

module wheel_and_bracket(anchor=CENTER, spin=0, orient=UP, caster=false, chain_port=false) {
    anchor_list = [
        named_anchor("driven-sprocket",[wheel_thickness/2+sprocket_ped_length+sprocket_thickness/2, 0, 0], orient=RIGHT),
        named_anchor("bracket-top", [-wheel_bracket_centers_offset(), -axle_x_offset_caster(caster), bracket_top_z_dist()], orient=UP),
        named_anchor("mount-plate", [-sprocket_alignment(), -motor_mount_v_dist, bracket_top_z_dist()], orient=UP),
        named_anchor("driven-side-bracket-top", [driven_side_bracket_dist(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("brake-side-bracket-top", [-brake_side_bracket_dist(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("wheel-center-bracket-top", [0, 0, bracket_top_z_dist()]),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, anchors=anchor_list) {
        simulated_wheel_and_sprocket() {
            position("brake_ped_top")
            bracket_side(anchor="axle-hole", orient=RIGHT, spin=90, caster=caster) {
                position("strut-mount-hole1")
                bracket_strut(anchor="mount-hole1", orient=LEFT, chain_port=chain_port, caster=caster);
            }
            position("sprocket_top")
            translate([chain_space, 0, 0])
            mirror([1,0,0])
            bracket_side(anchor="axle-hole", orient=RIGHT, spin=90, caster=caster) {
                position("strut-mount-hole2")
                bracket_strut(anchor="mount-hole1", orient=LEFT);
            }
        }
        children();
    }
}

module wheel_bracket_and_motor_assembly(spin=0, orient=UP, anchor=CENTER, caster=false, chain_port=false) {
    anchor_list = [
        named_anchor("bracket-top", [-wheel_bracket_centers_offset(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("motor-mount-center", [motor_mount_center_offset()[0], motor_mount_center_offset()[1], bracket_top_z_dist()]),
    ];
    attachable(spin=spin, anchor=anchor, orient=orient, anchors=anchor_list) {
        wheel_and_bracket(caster=caster, chain_port=chain_port) {
            translate(motor_mount_center_offset())
            position("wheel-center-bracket-top")
            simulated_motor(anchor="mount-plate", orient=DOWN, spin=180){
                position("sprocket")
                simulated_chain(anchor="sprocket-center", orient=RIGHT, spin=-chain_angle(), sprocket_to_sprocket_dist=sprocket_to_sprocket_dist());
            }
        }
        children();
    }
}

module wheel_bracket_motor_top_plate_assembly(anchor=CENTER, spin=0, orient=UP, top_plate_z_offset=0, caster=false, chain_port=false) {
    echo(str("top_plate_z_offset=", top_plate_z_offset));
    anchor_list = [
        named_anchor("bracket-top", [wheel_bracket_centers_offset(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("motor-mount-slot-center", [motor_mount_center_offset()[0], motor_mount_center_offset()[1], bracket_top_z_dist()]),
    ];
    attachable(spin=spin, anchor=anchor, orient=orient, anchors=anchor_list, size=[brake_side_bracket_dist() + driven_side_bracket_dist() + bracket_thickness, top_plate_size()[1], top_plate_size()[2]]) {
        wheel_bracket_and_motor_assembly(caster=caster, chain_port=chain_port) {
            translate([0,0,top_plate_z_offset])
            position("bracket-top")
            top_plate("wheel-mount-holes-center");
            position("motor-mount-center")
            translate([0,-5,top_plate_z_offset])
            motor_top_plate(motor_mount_center_offset=[motor_mount_center_offset()[0]+wheel_bracket_centers_offset(), motor_mount_center_offset()[1]], anchor="motor-mount-center");
        }
        children();
    }
}

module wheel_bracket_and_top_plate()
{
    wheel_and_bracket()
        position("bracket-top")
        translate([0, 0, 20])
        top_plate("wheel-mount-holes-center");
}

module wheel_and_motor()
{
    simulated_wheel_and_sprocket()
    translate(motor_mount_center_offset())
    translate([0,0,bracket_top_z_dist()])
    simulated_motor(anchor="mount-plate", orient=DOWN, spin=180);
}

// wheel_bracket_motor_top_plate_assembly(top_plate_z_offset=plywood_thickness);

module cart_bucket(spin=0, anchor=CENTER, orient=UP) {
    anchor_list= [
        named_anchor("top-of-bottom", [0, 0, (-(cart_side_size[0]+plywood_thickness)/2)+plywood_thickness], orient=UP),
    ];
    attachable(spin=spin, anchor=anchor, orient=UP, size=[cart_width, cart_length, cart_side_size[0]+plywood_thickness], anchors=anchor_list) {
        hsv(h=42, s=0.88, v=0.61, a=0.5)
        translate([0, 0, -(cart_side_size[0]+plywood_thickness)/2])
        cube(cart_bottom_size, anchor=BOTTOM) {
            position(TOP+LEFT)
            cube(cart_side_size, anchor=TOP+LEFT, orient=LEFT);
            position(TOP+RIGHT)
            cube(cart_side_size, anchor=TOP+RIGHT, orient=RIGHT);
        }
        children();
    }
}

module wheel_bracket_top_plate_no_motor(anchor=CENTER, spin=0, orient=UP, caster=false, top_plate_z_offset=0)
{
    anchor_list = [
        named_anchor("bracket-top", [-wheel_bracket_centers_offset(), -axle_x_offset_caster(caster), bracket_top_z_dist()]),
    ];
    attachable(spin=spin, anchor=anchor, orient=orient, anchors=anchor_list) {
        wheel_and_bracket(chain_port=false, caster=caster) {
            position("bracket-top")
            translate([0,0,top_plate_z_offset])
            top_plate("wheel-mount-holes-center");
        }
        children();
    }
}

cart_bucket(anchor=BOTTOM) {
    for (i=[-10, 10]) {
        position(BOTTOM)
        translate([i * INCH, 16 * INCH, 0])
        wheel_bracket_motor_top_plate_assembly(top_plate_z_offset=plywood_thickness,anchor="bracket-top", caster=false, chain_port=true);
    }
    for (i=[-10, 10]) {
        position(BOTTOM)
        translate([i * INCH, -16 * INCH, 0])
        wheel_bracket_top_plate_no_motor(top_plate_z_offset=plywood_thickness,anchor="bracket-top", spin=180, caster=true);
    }
}

// wheel_bracket_top_plate_no_motor(top_plate_z_offset=plywood_thickness, caster=true) show_anchors();

// wheel_and_bracket(caster=true);
// bracket_side(caster=true) show_anchors();
// bracket_strut(caster=false) show_anchors();
