include <BOSL2/std.scad>
include <rosetta-stone/std.scad>
include <BOSL2/screws.scad>

// rpi values
rpi_size = [56, 85, 5];
rpi_mount_hole_spacing = [49, 58];
rpi_mount_hole_offset = (rpi_size[1] - rpi_mount_hole_spacing[1])/2 - 3.5; // 3.5 mm from top corner
rpi_mount_hole_diam = 2.75;
rpi_mount_spec = screw_info("M2x0.4,5");
    
// motor driver values
motor_driver_size = [88.9, 69.85, 5];
motor_driver_mount_hole_spacing = [52.7, 63.5];
motor_driver_mount_hole_diam = 3.3;
motor_driver_mount_spec = screw_info("M3x0.5,5");

// regulator values
regulator_size = [65, 53, 5];
regulator_mount_hole_spacing = [57, 0];
regulator_mount_hole_v_offset = 19;
regulator_mount_hole_v_offset_from_center = regulator_size[1]/2 - regulator_mount_hole_v_offset;
regulator_mount_hole_diam = 3.3;
regulator_mount_spec = motor_driver_mount_spec;

// main platform values
main_platform_size = [160, 200, 10];
main_platform_mount_hole_spacing = [146, 186];
main_platform_mount_hole_diam = 6;

$fn=20;

board_float = 10;
diff("rpi-mount-holes motor-controller-mount-holes regulator-mount-holes")
simulated_4_hole_board(
    size=main_platform_size,
    mount_hole_spacing=main_platform_mount_hole_spacing,
    mount_hole_diam=main_platform_mount_hole_diam) {

    //rpi
    position(TOP)
    translate([40, 40, board_float])
    color_this("blue") simulated_4_hole_board(
        size=rpi_size,
        mount_hole_spacing=rpi_mount_hole_spacing,
        mount_hole_diam=rpi_mount_hole_diam,
        mount_hole_offset=[0, rpi_mount_hole_offset],
        anchor=BOTTOM);
    // rpi standoffs and mount holes
    position(TOP)
    translate([40, 40+rpi_mount_hole_offset, 0])
    standoffs4(rpi_mount_hole_spacing, 5, 10, anchor=BOTTOM, rounding=-2)
    tag("rpi-mount-holes")
    position(TOP)
    mount_threads4(rpi_mount_spec, rpi_mount_hole_spacing, anchor=TOP);

    // motor controller
    position(TOP)
    translate([0, -60, board_float])
    color_this("red") simulated_4_hole_board(
        size=motor_driver_size,
        mount_hole_spacing=motor_driver_mount_hole_spacing,
        mount_hole_diam=motor_driver_mount_hole_diam,
        anchor=BOTTOM);

    // motor controller standoffs and mount holes
    position(TOP)
    translate([0, -60, 0])
    standoffs4(motor_driver_mount_hole_spacing, 5,10, anchor=BOTTOM, rounding=-2)
    tag("motor-controller-mount-holes")
    position(TOP)
    mount_threads4(motor_driver_mount_spec, motor_driver_mount_hole_spacing, anchor=TOP);

    // regulator
    position(TOP)
    translate([-30, 40, board_float])
    color_this("green") simulated_4_hole_board(
        size=regulator_size,
        mount_hole_spacing=regulator_mount_hole_spacing,
        mount_hole_offset=[0,regulator_mount_hole_v_offset_from_center],
        hole_mask=[1,0,0,1],
        anchor=BOTTOM);

    // regulator standoffs and mount holes
    position(TOP)
    translate([-30, 40+regulator_mount_hole_v_offset_from_center, 0])
    standoffs4(
        size2d=regulator_mount_hole_spacing,
        l=5,
        d=10,
        standoff_mask=[1, 0, 0, 1],
        anchor=BOTTOM,
        rounding=-2)
    tag("regulator-mount-holes")
    position(TOP)
    mount_threads4(regulator_mount_spec, regulator_mount_hole_spacing, anchor=TOP, mount_hole_mask=[1, 0, 0, 1]);
}
