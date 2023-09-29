include <BOSL2/std.scad>
include <rosetta-stone/std.scad>

module simulated_regulator(size, mount_hole_h_spacing, mount_hole_v_offset, mount_hole_diam, anchor=CENTER, spin=0, orient=UP) {
    mount_hole_v_offset_from_center = size[1]/2 - mount_hole_v_offset;
    anchor_list=[
        named_anchor("mount_hole1", [-mount_hole_h_spacing/2, mount_hole_v_offset_from_center, 0]),
        named_anchor("mount_hole2", [mount_hole_h_spacing/2, mount_hole_v_offset_from_center, 0]),
    ];
    attachable(anchor=anchor, orient=orient, spin=spin, anchors=anchor_list, size=size) {
        diff("mount-holes"){
        cube(size, anchor=CENTER) {
            tag("mount-holes") {
            for (i=[-mount_hole_h_spacing/2,mount_hole_h_spacing/2]) {
                position(BACK)
                translate([i, -mount_hole_v_offset, 0])
                cyl(d=mount_hole_diam, l=size[2]+1, anchor=CENTER);
            }}
        }}
        children();
    }
}

// rpi values
rpi_width = 56;
rpi_height = 85;
rpi_thickness = 5;
rpi_mount_hole_v_spacing = 58;
rpi_mount_hole_h_spacing = 49;
rpi_mount_hole_offset = (rpi_height - rpi_mount_hole_v_spacing)/2 - 3.5; // 3.5 mm from top corner
rpi_mount_hole_diam = 2.75;
    
// motor driver values
motor_driver_width = 88.9;
motor_driver_height = 69.85;
motor_driver_thickness = 5;
motor_driver_mount_hole_h_spacing = 52.7;
motor_driver_mount_hole_v_spacing = 63.5;
motor_driver_mount_hole_diam = 3.3;

// regulator values
regulator_size = [65, 53, 5];
regulator_mount_hole_h_spacing = 57;
regulator_mount_hole_v_offset = 19;
regulator_mount_hole_diam = 3.3;

// main platform values
main_platform_size = [160, 200, 10];
main_platform_mount_hole_h_spacing = 146;
main_platform_mount_hole_v_spacing = 186;
main_platform_mount_hole_diam = 6;

board_float = 10;
diff("rpi-mount-holes motor-controller-mount-holes regulator-mount-holes")
simulated_4_hole_board(
    size=main_platform_size,
    mount_hole_h_spacing=main_platform_mount_hole_h_spacing,
    mount_hole_v_spacing=main_platform_mount_hole_v_spacing,
    mount_hole_diam=main_platform_mount_hole_diam) {

    //rpi
    *position(TOP)
    translate([40, 40, board_float])
    color_this("blue") simulated_4_hole_board(
        size=[rpi_width, rpi_height, rpi_thickness],
        mount_hole_h_spacing=rpi_mount_hole_h_spacing,
        mount_hole_v_spacing=rpi_mount_hole_v_spacing,
        mount_hole_diam=rpi_mount_hole_diam,
        mount_hole_v_offset=rpi_mount_hole_offset, anchor=BOTTOM);
    // rpi standoffs and mount holes
    position(TOP)
    translate([40, 40+rpi_mount_hole_offset, 0])
    standoffs4(rpi_mount_hole_h_spacing, rpi_mount_hole_v_spacing, 5, 10, anchor=BOTTOM, rounding=-2)
    tag("rpi-mount-holes")
    translate([0,0,-(main_platform_size[2])/2])
    mount_holes4(rpi_mount_hole_h_spacing, rpi_mount_hole_v_spacing, 5+main_platform_size[2]+1, rpi_mount_hole_diam, anchor=CENTER);

    // motor controller
    *position(TOP)
    translate([0, -60, board_float])
    color_this("red") simulated_4_hole_board(
        size=[motor_driver_width, motor_driver_height, motor_driver_thickness],
        mount_hole_h_spacing=motor_driver_mount_hole_h_spacing,
        mount_hole_v_spacing=motor_driver_mount_hole_v_spacing,
        mount_hole_diam=motor_driver_mount_hole_diam,
        anchor=BOTTOM);

    // motor controller standoffs and mount holes
    position(TOP)
    translate([0, -60, 0])
    standoffs4(motor_driver_mount_hole_h_spacing, motor_driver_mount_hole_v_spacing, 5,10, anchor=BOTTOM, rounding=-2)
    tag("motor-controller-mount-holes")
    translate([0,0,-(main_platform_size[2])/2])
    mount_holes4(motor_driver_mount_hole_h_spacing, motor_driver_mount_hole_v_spacing, 5+main_platform_size[2]+1, motor_driver_mount_hole_diam, anchor=CENTER);

    // regulator
    position(TOP)
    translate([-30, 40, board_float])
    color_this("green") simulated_regulator(
        size=regulator_size,
        mount_hole_h_spacing=regulator_mount_hole_h_spacing,
        mount_hole_v_offset=regulator_mount_hole_v_offset,
        mount_hole_diam=regulator_mount_hole_diam,
        anchor=BOTTOM)
        for (mount_position = ["mount_hole1", "mount_hole2"]) {
            position(mount_position)
            translate([0,0,-board_float - regulator_size[2]/2])
            cyl(d=10, l=5, anchor=BOTTOM, rounding1=-2)
                tag("regulator-mount-holes") {
                    position(CENTER)
                    translate([0,0,-main_platform_size[2]/2])
                    cyl(d=regulator_mount_hole_diam, l=5+main_platform_size[2]+1, anchor=CENTER);
                }
        }
}
