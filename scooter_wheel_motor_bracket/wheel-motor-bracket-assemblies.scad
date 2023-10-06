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

module wheel_and_bracket(anchor=CENTER, spin=0, orient=UP, caster=false) {
    anchor_list = [
        named_anchor("driven-sprocket",[wheel_thickness/2+sprocket_ped_length+sprocket_thickness/2, 0, 0], orient=RIGHT),
        named_anchor("bracket-top", [-wheel_bracket_centers_offset(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("mount-plate", [-sprocket_alignment(), -motor_mount_v_dist, bracket_top_z_dist()], orient=UP),
        named_anchor("driven-side-bracket-top", [driven_side_bracket_dist(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("brake-side-bracket-top", [-brake_side_bracket_dist(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("wheel-center-bracket-top", [0, 0, bracket_top_z_dist()]),
    ];
    attachable(anchor=anchor, spin=spin, orient=orient, anchors=anchor_list) {
        simulated_wheel_and_sprocket() {
            position("brake_ped_top")
            color_this("brown") bracket_side(anchor="axle-hole", orient=RIGHT, spin=90, caster=caster) {
                position("strut-mount-hole1")
                color_this("brown") bracket_strut(anchor="mount-hole1", orient=LEFT, chain_port=true, caster=caster);
            }
            position("sprocket_top")
            translate([chain_space, 0, 0])
            mirror([1,0,0])
            color_this("brown") bracket_side(anchor="axle-hole", orient=RIGHT, spin=90, caster=caster) {
                position("strut-mount-hole2")
                color_this("brown") bracket_strut(anchor="mount-hole1", orient=LEFT);
            }
        }
        children();
    }
}

module wheel_bracket_and_motor_assembly(spin=0, orient=UP, anchor=CENTER) {
    anchor_list = [
        named_anchor("bracket-top", [-wheel_bracket_centers_offset(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("motor-mount-center", [motor_mount_center_offset()[0], motor_mount_center_offset()[1], bracket_top_z_dist()]),
    ];
    attachable(spin=spin, anchor=anchor, orient=orient, anchors=anchor_list) {
        wheel_and_bracket() {
            translate(motor_mount_center_offset())
            position("wheel-center-bracket-top")
            simulated_motor(anchor="mount-plate", orient=DOWN, spin=180){
                position("sprocket")
                color_this("grey") simulated_chain(anchor="sprocket-center", orient=RIGHT, spin=-chain_angle(), sprocket_to_sprocket_dist=sprocket_to_sprocket_dist());
            }
        }
        children();
    }
}

module wheel_bracket_motor_top_plate_assembly(anchor=CENTER, spin=0, orient=UP, top_plate_z_offset=0) {
    echo(str("top_plate_z_offset=", top_plate_z_offset));
    anchor_list = [
        named_anchor("bracket-top", [wheel_bracket_centers_offset(), 0, bracket_top_z_dist()], orient=UP),
        named_anchor("motor-mount-slot-center", [motor_mount_center_offset()[0], motor_mount_center_offset()[1], bracket_top_z_dist()]),
    ];
    attachable(spin=spin, anchor=anchor, orient=orient, anchors=anchor_list, size=[brake_side_bracket_dist() + driven_side_bracket_dist() + bracket_thickness, top_plate_size()[1], top_plate_size()[2]]) {
        wheel_bracket_and_motor_assembly() {
            translate([0,0,top_plate_z_offset])
            position("bracket-top")
            color_this("brown") top_plate("wheel-mount-holes-center");
            position("motor-mount-center")
            translate([0,-5,top_plate_z_offset])
            color_this("brown") motor_top_plate(motor_mount_center_offset=[motor_mount_center_offset()[0]+wheel_bracket_centers_offset(), motor_mount_center_offset()[1]], anchor="motor-mount-center");
        }
        children();
    }
}

module wheel_bracket_and_top_plate()
{
    wheel_and_bracket()
        position("bracket-top")
        translate([0, 0, 20])
        color_this("brown") top_plate("wheel-mount-holes-center");
}

module wheel_and_motor()
{
    simulated_wheel_and_sprocket()
    translate(motor_mount_center_offset())
    translate([0,0,bracket_top_z_dist()])
    simulated_motor(anchor="mount-plate", orient=DOWN, spin=180);
}

// wheel_and_bracket();
// wheel_bracket_and_motor_assembly();
wheel_bracket_motor_top_plate_assembly(top_plate_z_offset=plywood_thickness);

// wheel_bracket_and_top_plate();
// top_plate();
// motor_top_plate(motor_mount_v_dist = motor_mount_v_dist, sprocket_alignment = sprocket_alignment(), motor_mount_center_offset=motor_mount_center_offset()) show_anchors();
// wheel_and_motor();
