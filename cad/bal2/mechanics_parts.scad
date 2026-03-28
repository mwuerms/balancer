/**
 * Martin Egli
 * 2022-09-14
 * mechanical parts
 */

use <parts.scad>
use <screws.scad>

module lego_axis_cut(len = 10) {
    hull() {
        translate([-1.0, -0.9, 0]) 
        cube([2.0, 1.8, len]);
        translate([-0.9, -1.0, 0]) 
        cube([1.8, 2.0, len]);
    }
    hull() {
        translate([-2.4, -0.7, 0]) 
        cube([4.8, 1.4, len]);
        translate([-2.2, -0.9, 0]) 
        cube([4.4, 1.8, len]);
    }
    hull() {
        translate([-0.9, -2.2, 0]) 
        cube([1.8, 4.4, len]);
        translate([-0.7, -2.4, 0]) 
        cube([1.4, 4.8, len]);
    }
}
lego_axis_cut();

module bldc5010_m3cut_rotor(show = 1, len = 10, loc_res = 32) {
    if(show) {
        origin();
    }
    r1 = (15+9.6)/2/2;
    for(n=[0:1:3]) {
        translate([r1*cos(n*90), r1*sin(n*90), 0]) {
            rotate([180, 0, 0])
            m3_cut(len = len, loc_res = loc_res);
        }
    }
}

module bldc5010_m3cut_rotor2(show = 1, len = 10, loc_res = 32) {
    if(show) {
        origin();
    }
    r1 = (15+9.6)/2/2;
    for(n=[0:1:1]) {
        translate([r1*cos(n*180), r1*sin(n*180), 0]) {
            rotate([180, 0, 0])
            m3_cut(len = len, loc_res = loc_res);
        }
    }
}

module bldc5010_m3cut_stator(show = 1, len = 10, loc_res = 32) {
    if(show) {
        origin();
    }
    r1 = (19+13.4)/2/2;
    for(n=[0:2:3]) {
        translate([r1*cos(n*90), r1*sin(n*90), 0]) {
            m3_cut(len = len, loc_res = loc_res);
        }
    }
    r2 = (22+16.6)/2/2;
    for(n=[1:2:3]) {
        translate([r2*cos(n*90), r2*sin(n*90), 0]) {
            m3_cut(len = len, loc_res = loc_res);
        }
    }
}

module bldc5010_motor(show = 1, loc_res = 32) {
    if(show) {
        origin(25);
        #translate([0, 0, 20])
        bldc5010_m3cut_rotor();
        #translate([0, 0, -2])
        bldc5010_m3cut_stator();
    }
    color("LightGray") {
        difference() {
            union() {
                // from bottom
                translate([0,0, 0])
                cylinder(d = 25, h = 2, $fn = loc_res);
                translate([0,0, 0])
                cylinder(d = 14.4, h = 19, $fn = loc_res);
                translate([0,0, 7])
                cylinder(d = 50, h = 10, $fn = loc_res);
                translate([0,0, 17])
                cylinder(d1 = 50, d2 = 18, h = 2, $fn = loc_res);
            }
            translate([0, 0, 20]) 
            bldc5010_m3cut_rotor();
            translate([0, 0, -2])
            bldc5010_m3cut_stator();
        }
    }
    color("DarkGray") {
        translate([0,0, -1.5])
        cylinder(d = 4, h = 24, $fn = loc_res);
    }
    translate([0, 0, 3])
    rotate([0, 0, 45]) {
        color("Red")
        translate([4, -2.5, 0])
        rotate([0, 90, 0])
        cylinder(d = 2.5, h=20, $fn = loc_res);
        color("Black")
        translate([4, 0, 0])
        rotate([0, 90, 0])
        cylinder(d = 2.5, h=20, $fn = loc_res);
        color("Yellow")
        translate([4, +2.5, 0])
        rotate([0, 90, 0])
        cylinder(d = 2.5, h=20, $fn = loc_res);
    }
}

module bldc5010_magnet_holder_5mm_v1_0(loc_res = 32) {
    difference() {
        translate([0, 0, 0])
        cylinder(d = 6, h = 4, $fn = loc_res);

        // magnet dia 4 x 2mm 
        translate([0, 0, -1])
        cylinder(d = 4.1, h = 4, $fn = loc_res);

        // axis, 4 mm
        translate([0, 0, -1])
        cylinder(d = 4.1, h = 10, $fn = loc_res);
    }
}

module ballbearing(odia = 20, idia = 10, th = 4, edge = 3, col = "Silver", loc_res = 32) {
    color(col)
    difference() {
        translate([0, 0, 0])
        cylinder(d = odia, h = th, $fn = loc_res);
        translate([0, 0, -1])
        cylinder(d = odia-edge, h = th+2, $fn = loc_res);
    }
    color(col)
    difference() {
        translate([0, 0, 0])
        cylinder(d = idia+edge, h = th, $fn = loc_res);
        translate([0, 0, -1])
        cylinder(d = idia, h = th+2, $fn = loc_res);
    }
    color("DimGray")
    difference() {
        translate([0, 0, 0.1])
        cylinder(d = odia-edge, h = th-0.2, $fn = loc_res);
        translate([0, 0, -1])
        cylinder(d = idia+edge, h = th+2, $fn = loc_res);
    }
}

module ballbearing18x12x4(loc_res = 32) {
    ballbearing(odia = 18, idia = 12, th = 4, edge = 2, loc_res = loc_res);
}

module ballbearing27x20x4(loc_res = 32) {
    ballbearing(odia = 27, idia = 20, th = 4, edge = 2, loc_res = loc_res);
}

module ballbearing55x45x6(loc_res = 32) {
    ballbearing(odia = 55, idia = 45, th = 6, edge = 3, loc_res = loc_res);
}

module roller_bearing(dia1 = 3, dia2 = 5, h1 = 4*5, h2 = 5, h3 = 2*5, loc_res = 32) {
    color("Silver")
    translate([0, 0, 0])
    cylinder(d = dia1, h = h1, $fn = loc_res);
    color("PaleGoldenrod")
    translate([0, 0, h2])
    cylinder(d = dia2, h = h3, $fn = loc_res);
}
//bldc5010_motor();
