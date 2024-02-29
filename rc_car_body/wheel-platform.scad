include <rosetta-stone/boards.scad>
include <BOSL2/std.scad>
include <BOSL2/screws.scad>
include <BOSL2/gears.scad>
include <rosetta-stone/std.scad>
include <rosetta-stone/ball-bearings.scad>

// main platform values
main_platform_size = [160, 200, 10];
main_platform_mount_hole_spacing = [146, 186];
main_platform_mount_hole_diam = 6;

motor_diam = 40;
motor_mount_spacing = 25;
motor_washer_diam = 21;
dist_between_motor_centers=85+motor_diam;
// echo(str("dist between motor centers", dist_between_motor_centers));
wheel_mount_thickness = 7;
wheel_mount_to_frame_hole_dist = 40;
wheel_mount_diam = 70;

$fn=30;

mod=0.81;
// 16 tooth gear specs
//        teeth, module, pressure angle
// gear_16 = [16, mod, 20]; 
// gear_19 = [19, mod, 20];

dist_between_wheel_mount_center_platform_mount = [
    (main_platform_size[0]-main_platform_mount_hole_spacing[0]-wheel_mount_thickness)/2,
    (main_platform_mount_hole_spacing[1]-dist_between_motor_centers)/2];

function dist_between_points(p1, p2) = sqrt(((p1[0]-p2[0])^2 + (p1[1]-p2[1])^2));
helical_ang=20;
gear_data = planetary_gears(mod=mod, n=3, max_teeth=75, ring_sun=1/4, helical=helical_ang);
// echo(gear_data);
// echo(gear_data[2][4]);
// echo(str("dist p1->p2", dist_between_points(gear_data[2][4][0], gear_data[2][4][1])));
// echo(str("dist p2->p3", dist_between_points(gear_data[2][4][1], gear_data[2][4][2])));
// echo(str("dist p3->p1", dist_between_points(gear_data[2][4][2], gear_data[2][4][0])));

// bearing data
wheel_thickness = 8;
roll_portion = 0.2; // percentage of ball exposed to be rolled on
channel_bearing_diam=6.25;
channel_radius=40;
fill_plug_screw_name = "M7-1";
bearing_diam = 6;
exposed_ball_portion = roll_portion * channel_bearing_diam;
fill_plug_flat_height = 3.5;
flat_diam = channel_bearing_diam-1;
threaded_portion = wheel_thickness - (channel_bearing_diam-exposed_ball_portion)-(fill_plug_flat_height-channel_bearing_diam/2);

// function planetary_gears_set_sun(n, sun_teeth, helical=0, circ_pitch, mod, diam_pitch,
//                          ring_carrier, carrier_ring, sun_carrier, carrier_sun, sun_ring, ring_sun,
//                          gear_spin=0) =
//     let(
//         mod = module_value(mod=mod,circ_pitch=circ_pitch,diam_pitch=diam_pitch),
//         dummy = one_defined([ring_carrier,carrier_ring,sun_carrier,carrier_sun,sun_ring,ring_sun],
//                             "ring_carrier,carrier_ring,sun_carrier,carrier_sun,sun_ring,ring_sun"),
//         // ratio is between the sun and ring 
//         ratio = is_def(ring_carrier) ? assert(is_finite(ring_carrier) && ring_carrier>1 && ring_carrier<2, "ring/carrier ratio must be between 1 and 2")
//                                        ring_carrier - 1
//               : is_def(carrier_ring) ? assert(is_finite(carrier_ring) && carrier_ring>1/2 && carrier_ring<1, "carrier/ring ratio must be between 1/2 and 1")
//                                        1/carrier_ring - 1
//               : is_def(sun_carrier) ?  assert(is_finite(sun_carrier) && sun_carrier>2, "sun/carrier ratio must be larger than 2")
//                                        1/(sun_carrier-1)
//               : is_def(carrier_sun) ?  assert(is_finite(carrier_sun) && carrier_sun<1/2, "carrier/sun ratio must be smaller than 1/2")
//                                        1/(1/carrier_sun-1)
//               : is_def(sun_ring) ?     assert(is_finite(sun_ring) && abs(sun_ring)>1, "abs(sun/ring) ratio must be larger than 1")
//                                        1/abs(sun_ring)
//               : /*is_def(ring_sun)*/   assert(is_finite(ring_sun) && abs(ring_sun)<1, "abs(ring/sun) ratio must be smaller than 1")
//                                        abs(ring_sun),
//         pq = rational_approx(ratio, max_teeth),
//         factor = floor(max_teeth/pq[1]),
//         temp_z_sun = factor*pq[0],
//         temp_z_ring = factor*pq[1],
//         z_sun = temp_z_sun%2==0 ? temp_z_sun+1 : temp_z_sun,
//         z_ring = temp_z_ring%2==0 ? min(temp_z_ring+1, max_teeth-(max_teeth%2==0?1:0)) : temp_z_ring,
//         z_planet = (z_ring-z_sun)/2
//     )
//     assert(z_planet==floor(z_planet),"Planets have non-integer teeth count!  Algorithm failed.")
//     let(
//         d12 = gear_dist(mod=mod,z_sun,z_planet,helical),
//         ps_sun = auto_profile_shift(teeth=z_sun,helical=helical),
//         ps_planet = auto_profile_shift(teeth=z_planet,helical=helical),
//         ps_ring = ps_sun+2*ps_planet,
//         ring_spin = ring_sun || ring_carrier ? gear_spin
//                   : sun_ring ? -gear_spin*z_sun/z_ring
//                   : carrier_ring ? gear_spin*(z_ring+z_sun)/z_ring
//                   : 0,
//         planet_rot = ring_carrier ? gear_spin*z_ring/(z_ring+z_sun)
//                    : carrier_sun || carrier_ring ? gear_spin
//                    : sun_carrier ? gear_spin*z_sun/(z_ring+z_sun)
//                    : carrier_ring ? gear_spin*z_ring/(z_ring+z_sun)
//                    : 0,
//         sun_spin = ring_sun ? -gear_spin*z_ring/z_sun
//                  : sun_ring || sun_carrier ? gear_spin
//                  : carrier_sun ? (z_ring+z_sun)*gear_spin/z_sun
//                  : 0,
//         planet_spin = -sun_spin*z_sun/z_planet,

//         quant = 360/(z_sun+z_ring),
//         planet_angles = [for (uang=lerpn(0,360,n,endpoint=false)) quant(uang,quant)+planet_rot],
//         planet_pos = [for(ang=planet_angles) d12*[cos(ang),sin(ang)]],
//         planet_spins = [for(ang=planet_angles) (z_sun/z_planet)*(ang-90)+90+ang+360/z_planet/2+planet_spin],

//         final_ratio = ring_carrier ? 1+z_sun/z_ring
//                     : carrier_ring ? 1/(1+z_sun/z_ring)
//                     : sun_carrier ? 1+z_ring/z_sun
//                     : carrier_sun ? 1/(1+z_ring/z_sun)
//                     : sun_ring ? z_ring/z_sun
//                     : /* ring_run */ z_sun/z_ring
//    )   
//    [  
//      ["sun", z_sun, ps_sun, sun_spin],
//      ["ring", z_ring, ps_ring, 360/z_ring/2 * (1-(z_sun%2))+ring_spin],
//      ["planets", z_planet, ps_planet, planet_spins, planet_pos, planet_angles],
//      ["ratio", final_ratio]
//    ];

module gear_chamf(id, anchor=CENTER, orient=UP) {
    len = 10;
    // echo(len);
    // echo(id);
    attachable(spin=0, orient=orient, anchor=anchor, size=[id,id,len]) {
        translate([0, 0, 3])
        diff() {
            tube(id=id, l=len, wall=10)
            position(TOP)
            tag("remove") cyl(chamfer2=-3, d=id, l=len, anchor=TOP, chamfang=50);
        }
        children();
    }
}

module planet_gear(spin=0, helical=-helical_ang) {
    pos_height=8;
    neg_height=2;
    total_height =pos_height+neg_height;
    diff("remove other") {
        spur_gear(mod=mod, teeth=gear_data[2][1], profile_shift=gear_data[2][2], gear_spin=spin, thickness=pos_height, helical=helical) {
            translate([0, 0, pos_height])
            spur_gear(mod=mod, teeth=gear_data[2][1], profile_shift=gear_data[2][2], gear_spin=spin, thickness=pos_height, helical=helical) {
                // remove top part
                tag("other") position(TOP) cyl(d=50, l=pos_height-neg_height, anchor=TOP);
                // axle hole
                translate([0, 0, -pos_height])
                tag("other") cyl(d=4, l=total_height+1);
                // bearing hole
                translate([0, 0, -1])
                tag("other") cyl(d=7, l=total_height/2+1, anchor=TOP);
            // position(TOP) screw_hole("M3x0.5", l=10, anchor=TOP, thread=false);
            }
            tag("other") {
                position(BOTTOM)
                gear_chamf(id=20, anchor=TOP);
            //     position(TOP)
            //     gear_chamf(id=20, anchor=TOP, orient=DOWN);
            }
        }
    }
}

module alignment_tool() {
    height=3;
    positions = [
        [19.9546, 0],
        [-9.97731, -17.2812],
        [-9.97731, 17.2812]];
    // path=gear_data[2][4];
    // stroke(path, closed=true);
    // difference() {
    //     translate([0, 0,-height/2])
    //     scale([1.5, 1.5, 1]) offset_sweep(round_corners(path, r=5), h=height);
    //     translate([0, 0,-height/2])
    //     scale([0.8, 0.8, 1]) offset_sweep(round_corners(path, r=5), h=height);
    //     move_copies(gear_data[2][4]) screw_hole("M3x0.5", thread=true, l=5);
    // }
    
    difference() {
        hull()
        // zrot_copies(n=3, sa=0, r=gear_data[4][1]) cyl(d=8, l=height, anchor=CENTER);
        move_copies(gear_data[2][4]) cyl(d=8, l=height, anchor=CENTER);
        scale([0.6, 0.6, 1]) hull() zrot_copies(n=3, sa=0, r=gear_data[4][1]) cyl(d=10, l=height, anchor=CENTER);
        move_copies(gear_data[2][4]) screw_hole("M3x0.5", thread=true, l=5);
        
        // zrot_copies(n=3, sa=0, r=gear_data[4][1]) screw_hole("M3x0.5", thread=true, l=5);
    }
}

module wheel_bearing_fill_plug()
{
    fill_plug(
        screw_name=fill_plug_screw_name,
        threaded_length=threaded_portion,
        nonthreaded_length=fill_plug_flat_height,
        nonthreaded_diam=channel_bearing_diam,
        channel_bearing_diam=channel_bearing_diam,
        channel_radius=channel_radius,
        internal=false,
        anchor=BOTTOM,
        orient=DOWN);
}

module ring_gear_wheel() {
    // echo("scre-w_length", screw_length);
    // echo("non_threaded_length
    diff("ring-gear-removals") {
        ring_gear(mod=mod, teeth=gear_data[1][1], profile_shift=gear_data[1][2], gear_spin=gear_data[1][3],backing=15,thickness=wheel_thickness, helical=helical_ang)
            tag("ring-gear-removals")
            position(TOP)
            translate([0, 0, roll_portion*channel_bearing_diam])
            bearing_channel(
                channel_radius=channel_radius,
                bearing_diam=channel_bearing_diam, anchor=TOP)
                position("fill-hole")
                fill_plug(
                    fill_plug_screw_name,
                    threaded_portion,
                    fill_plug_flat_height,
                    channel_bearing_diam,
                    channel_bearing_diam,
                    channel_radius,
                    internal=true,
                    anchor=BOTTOM,
                    orient=DOWN);
    }
    
}

module gearset(anchor=CENTER) {
    diff("remove") {
        spur_gear(mod=mod, teeth=gear_data[0][1], profile_shift=gear_data[0][2], gear_spin=gear_data[0][3], thickness=10, helical=helical_ang, anchor=anchor) {  //sun
            tag("remove") position(CENTER) cyl(d=3.25, l=11, anchor=CENTER);
            rotate([0,0,30])
            move_copies(gear_data[2][4]) {
                // color("red")
                planet_gear(gear_data[2][3][$idx]);
            }
            ring_gear_wheel();
        }
    }
}

module wheel_mount(show_gearset=false, spin=0, orient=UP, anchor=CENTER, platform_mount_right=true) {
    height_from_platform = 25;
    attachable(spin=spin, orient=orient, anchor=anchor, size=[wheel_mount_diam, wheel_mount_thickness, height_from_platform*2]) {
        rotate([90, 0, 0])
        diff("mount-holes") {
            cyl(d=wheel_mount_diam,l=wheel_mount_thickness, anchor=CENTER) {
                cube([wheel_mount_diam, height_from_platform, wheel_mount_thickness], anchor=FRONT) {
                    tag("mount-holes") position(BACK) cube([100, 50, wheel_mount_thickness], anchor=FRONT);
                    tag("mount-holes") position(BACK) xcopies(n=2, spacing=wheel_mount_to_frame_hole_dist) screw_hole("M3x0.5", thread=true, l=10, anchor=TOP, orient=BACK);
                    position(BACK)
                    translate([
                              dist_between_wheel_mount_center_platform_mount[1] * (platform_mount_right ? 1 : -1),
                              0,
                              0])
                    tag("mount-holes") cube([10,20,100], anchor=TOP, orient=BACK);
                }
                tag("mount-holes") cyl(d=motor_washer_diam, l=wheel_mount_thickness+1);
                position(TOP)
                xcopies(n=2, spacing=motor_mount_spacing)
                tag("mount-holes") screw_hole("M3x0.5", head="socket", l=wheel_mount_thickness+1, thread=false, anchor=TOP);
                position(TOP)
                rotate([0,0,30])
                move_copies(gear_data[2][4]) {
                    tag("mount-holes")
                    
                    screw_hole("M3x0.5", l=10, thread=true, anchor=TOP);
                }
                if (show_gearset) {
                    position(TOP)
                    gearset(anchor=BOTTOM);
                }
            }
        }
        children();
    }
}

module test_build_gears() {
    // todo take drive gears off of motors and create my own
    // they can now be helical. basically use the coffee grinder thing I have
    helical=20;
    gear_data = planetary_gears(mod=mod, n=3, max_teeth=150, ring_sun=1/4, helical=helical);
    // echo(gear_data);
    // spur_gear(mod=mod, teeth=gear_data[0][1], profile_shift=gear_data[0][2], gear_spin=gear_data[0][3], thickness=10, helical=helical);  //sun
        
    // move_copies(gear_data[2][4])
    // color("red") spur_gear(mod=mod, teeth=gear_data[2][1], profile_shift=gear_data[2][2], gear_spin=gear_data[2][3][$idx], thickness=10, helical=-helical);
    // ring_gear(mod=mod, teeth=gear_data[1][1], profile_shift=gear_data[1][2], gear_spin=gear_data[1][3],backing=5,thickness=10, helical=helical);
        
    wheel_mount(true);
    
}

module platform(spin=0, orient=UP, anchor=CENTER) {
    attachable(spin=spin, orient=orient, anchor=anchor, size=main_platform_size) {
        diff() {
            simulated_4_hole_board(
                size=main_platform_size,
                mount_hole_spacing=main_platform_mount_hole_spacing,
                mount_hole_diam=main_platform_mount_hole_diam) {
                // add the wheel mounts on the right side
                let(wheel_mount_positions = [
                    [RIGHT, 90],
                    [LEFT, -90],
                ])
                for (positions = wheel_mount_positions) {
                    tag("remove")
                    ycopies(n=2, spacing=dist_between_motor_centers) {
                        position(positions[0]+TOP)
                        translate([positions[0] == RIGHT ? -wheel_mount_thickness/2 : wheel_mount_thickness/2, 0, 0] + TOP)
                        ycopies(n=2, spacing=wheel_mount_to_frame_hole_dist)
                        screw_hole("M3x0.5", l=10, thread=false, head="socket", anchor=TOP);
                        // position(positions[0]+BOTTOM)
                        // translate([0, 0, 2])
                        // cube([wheel_mount_thickness, wheel_mount_diam, 10], anchor=TOP + positions[0]);
                    }
                    tag("remove")
                    for (i=[-1, 1])
                    {
                        translate([0, dist_between_motor_centers/2 * i, 0])
                        position(positions[0]+BOTTOM) linear_extrude(2) projection(cut=false) wheel_mount(anchor=BOTTOM+FRONT, spin=positions[1], platform_mount_right= (i==-1 && positions[0] == LEFT) || (i==1 && positions[0] == RIGHT));
                    }
                }
                move_copies([[10, 0, 0], [2, 0, 0], [-6, 0, 0]])
                position(BOTTOM)
                tag("remove") mount_threads4("M4x0.7", size=[1*INCH, 1*INCH, main_platform_size[2]], anchor=TOP+RIGHT, orient=DOWN); 
                tag("remove") position(RIGHT) translate([-10, 0, 0]) cube([25, 80, main_platform_size[2]], anchor=RIGHT);
                tag("remove") position(LEFT) translate([10, 0, 0]) cube([50, 80, main_platform_size[2]], anchor=LEFT);
            }
        }
        children();
    }
}

module build_wheel_platform(spin=0, orient=UP, anchor=CENTER) {
    platform(spin, orient, anchor) {
        // add the wheel mounts on the right side
        // let(wheel_mount_positions = [
        //     [RIGHT, 90],
        //     [LEFT, -90],
        // ])
        // for (positions = wheel_mount_positions) {
        //     ycopies(n=2, spacing=dist_between_motor_centers)
        //     position(positions[0]+BOTTOM)
        //     wheel_mount(show_gearset=false, anchor=TOP+FRONT, spin=positions[1]);
        // }
        translate([0, dist_between_motor_centers/2, 0])
        position(RIGHT+BOTTOM)
        wheel_mount(show_gearset=false, anchor=TOP+FRONT, spin=90, platform_mount_right=true);
    }
}

// build_wheel_platform();
// platform();

// test_build_gears();
wheel_mount(show_gearset=true, platform_mount_right=false);
// gearset();
// ring_gear_wheel();
// planet_gear(helical=-helical_ang);
// gear_chamf();
// alignment_tool();

// projection(cut=false) wheel_mount(anchor=BOTTOM);
// wheel_bearing_fill_plug();
