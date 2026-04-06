/**
 * Martin Egli
 * 2022-09-14
 * electronics
 */

use <parts.scad>

module lipo500mAh(col = "LightGray", loc_res = 32) {
    color(col) {
        hull() {
            translate([0, 0, 0])
            cylinder(d = 16, h = 32, $fn = loc_res);
            translate([-(62-16/2), +7, 0])
            cylinder(d = 2, h = 32, $fn = loc_res);
            translate([-(62-16/2), -7, 0])
            cylinder(d = 2, h = 32, $fn = loc_res);
        }
    }
    color("Red") {
        translate([0, -6, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, -4, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
    }
    color("Black") {
        translate([0, -2, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, -0, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, +2, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, +4, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, +6, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
    }
}
//lipo500mAh();
module lipo500mAh_cut(loc_res = 32) {
    translate([-(64-16/2), -17/2, 0])
    cube([72, 17, 34]);
}
//lipo500mAh_cut();

module rockswitch(col = "DarkGray", loc_res = 32) {
    color(col) {
        translate([2, -11/2, 4])
        rotate([0, -30, 0])
        cube([4, 11, 8]);
        translate([0, -15/2, 0])
        cube([2, 15, 21]);
        translate([-12, -13/2, 0])
        cube([12, 13, 21]);
        translate([-20, -1/2, (21-16)/2])
        cube([12, 1, 16]);
    }
}
//rockswitch();

module rockswitch_cut(loc_res = 32) {
    translate([-20, -13.4/2, -0.2])
    cube([25, 13.4, 21.4]);
}
//rockswitch_cut();

module google_pxl7a_mobilephone(col = "LightBlue", loc_res = 32) {
    color(col)
    hull() {
        translate([+70/2, +140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
        translate([+70/2, -140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
        translate([-70/2, +140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
        translate([-70/2, -140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
    }
    // USBC
    hull() {
        translate([+5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
        translate([-5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
    }
}
//google_pxl7a_mobilephone();

module samsung_mobilephone(col = "DarkGray", loc_res = 32) {
    color(col)
    hull() {
        translate([+((74-10)/2), +((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
        translate([+((74-10)/2), -((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
        translate([-((74-10)/2), +((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
        translate([-((74-10)/2), -((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
    }
    // USBC
    hull() {
        translate([+5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
        translate([-5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
    }
}
//samsung_mobilephone();

module generic_pcb(col = "LightGreen", wid = 100, len = 160, th = 1.5) {
    color(col)
    translate([0, 0, 0])
    cube([wid, len, th]);
}
//PTV09A-4030F

module poti_PTV09A_4(loc_res = 32) {
    // knob
    color("DarkGray") {
        cylinder(d = 6.8, h = 15, $fn = loc_res);
    }
    // ody
    color("LightBlue") {
        translate([-10/2, -10/2, 0])
        cube([10, 12, 6.8]);
    }
    // contacts
    color("Silver") {
        translate([-(10.6-2)/2, 0, -3.8])
        cylinder(d = 2, h = 6.8, $fn = loc_res);
        translate([+(10.6-2)/2, 0, -3.8])
        cylinder(d = 2, h = 6.8, $fn = loc_res);
        
        translate([+(5)/2, 7, -3.8])
        cylinder(d = 1, h = 6.8, $fn = loc_res);
        translate([0, 7, -3.8])
        cylinder(d = 1, h = 6.8, $fn = loc_res);
        translate([-(5)/2, 7, -3.8])
        cylinder(d = 1, h = 6.8, $fn = loc_res);
    }
}
//poti_PTV09A_4();

module poti_PTV09A_4_cut(loc_res = 32) {
    translate([-(10.6-2)/2, 0, -5])
    cylinder(d = 2.5, h = 10, $fn = loc_res);
    translate([+(10.6-2)/2, 0, -5])
    cylinder(d = 2.5, h = 10, $fn = loc_res);
    
    translate([+(5)/2, 7, -5])
    cylinder(d = 1.1, h = 10, $fn = loc_res);
    translate([0, 7, -5])
    cylinder(d = 1.1, h = 10, $fn = loc_res);
    translate([-(5)/2, 7, -5])
    cylinder(d = 1.1, h = 10, $fn = loc_res);
}
//poti_PTV09A_4_cut();

module soic8(col1 = "DarkGray", col2 = "LightGray") {
    color(col1) {
        difference() {
            union() {
                translate([-5/2, -4/2, 0])
                cube([5, 4, 1.75]);
            }
            hull() {
                translate([-5.2/2, -4.4/2, 1])
                cube([5.2, 0.1, 1]);
                translate([-5.2/2, -4.1/2, 2])
                cube([5.2, 1, 0.1]);
            }
        }
    }
    color(col2) {
        translate([-3*1.27/2-0.4/2, -6/2, 0.01])
        cube([0.4, 6, 0.6]);
        translate([-1*1.27/2-0.4/2, -6/2, 0.01])
        cube([0.4, 6, 0.6]);
        translate([+1*1.27/2-0.4/2, -6/2, 0.01])
        cube([0.4, 6, 0.6]);
        translate([+3*1.27/2-0.4/2, -6/2, 0.01])
        cube([0.4, 6, 0.6]);
    }
}
soic8();
