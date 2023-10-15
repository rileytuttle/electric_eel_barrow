include <rosetta-stone/ball-bearings.scad>
include <rosetta-stone/boards.scad>
include <common-dims.scad>

$fn=30;

// module caster_plate() {
//     plate_size=[bracket_width, bracket_width, 10];
//     roll_portion = 0.2; // percentage of ball exposed to be rolled on
//     exposed_ball_portion = roll_portion * channel_bearing_diam;
//     flat_diam = channel_bearing_diam-1;
//     threaded_portion = plate_size[2] - (channel_bearing_diam-exposed_ball_portion)-(bearing_fill_plug_screw_flat_height-channel_bearing_diam/2);
//     diff("remove") {
//     color_this("brown") bearing_plate(
//         plate_size=plate_size,
//         channel_bearing_diam=channel_bearing_diam,
//         exposed_ball_portion=exposed_ball_portion,
//         channel_radius=channel_radius,
//         fill_plug_flat_height=bearing_fill_plug_screw_flat_height,
//         fill_plug_screw_name=bearing_fill_plug_screw_name) {
//         position("bearing-center")
//         sim_bearings(channel_radius, bearing_diam, n_bearings);
//         tag("remove") {
//             position(CENTER)
//             mount_holes4([80, 80], 10, 5);
//         }
//     }}
    
// }

// caster_plate();
// bearing();
// example_bearing_plate();

// fill_plug(
//     bearing_fill_plug_screw_name,
//     10,
//     10,
//     6,
//     6.25,
//     10,
//     internal=true);
    
bearing_plate(
    [30, 30, 10],
    6.25,
    1.5,
    10,
    4,
    "M7-1");

