/**
 * Martin Egli
 * 2026-02-22
 */
use <leg.scad>
use <electronics_parts.scad>

module sidepanel_left(loc_res = 32) {
    difference() {
        union() {
            hull() {
                translate([16, 87, -3])
                cylinder(d = 6, h = 3, $fn = loc_res);
                translate([16, 6, -3])
                cylinder(d = 6, h = 3, $fn = loc_res);
                translate([10, 0, -3])
                cylinder(d = 6, h = 3, $fn = loc_res);
                translate([-1, 87, -3])
                cylinder(d = 6, h = 3, $fn = loc_res);
                translate([-1, 6, -3])
                cylinder(d = 6, h = 3, $fn = loc_res);
                translate([-1, 0, -3])
                cylinder(d = 6, h = 3, $fn = loc_res);
            }
            // ruber band mount
            translate([0, 20, 0]) {
                hull() {
                    translate([18, 0, -7])
                    cylinder(d = 2, h = 5, $fn = loc_res);
                    translate([18, 4, -7])
                    cylinder(d = 2, h = 5, $fn = loc_res);
                }
                hull() {
                    translate([18, -2, -7])
                    cylinder(d = 2, h = 2, $fn = loc_res);
                    translate([18, 6, -7])
                    cylinder(d = 2, h = 2, $fn = loc_res);
                }
            }
            translate([0, 75, 0]) {
                hull() {
                    translate([18, 0, -7])
                    cylinder(d = 2, h = 5, $fn = loc_res);
                    translate([18, 4, -7])
                    cylinder(d = 2, h = 5, $fn = loc_res);
                }
                hull() {
                    translate([18, -2, -7])
                    cylinder(d = 2, h = 2, $fn = loc_res);
                    translate([18, 6, -7])
                    cylinder(d = 2, h = 2, $fn = loc_res);
                }
            }
        }
        // cut m3 holes leg mount
        translate([16, 6, -8])
        cylinder(d = 3.2, h = 10, $fn = loc_res);
        translate([0, 6, -8])
        cylinder(d = 3.2, h = 10, $fn = loc_res);
        translate([0, 0, -8])
        cylinder(d = 3.2, h = 10, $fn = loc_res);
        // cut m3 holes on top
        translate([16, 87, -8])
        cylinder(d = 3.2, h = 10, $fn = loc_res);
        translate([0, 87, -8])
        cylinder(d = 3.2, h = 10, $fn = loc_res);
        // cut hole for lipo battery
        translate([0, 9, -8])
        cube([17, 34, 12]);
        // cut hole for switch
        translate([0, 50, -8])
        cube([17, 34, 12]);
    }
}
 
module elements(show_elements = 1, loc_res = 32) {
    if(show_elements) {
        translate([-34, +2, 77])
        rotate([0, 90, 0])
        pcbMPU9250();
        
        *translate([-34, 0, 50])
        rotate([90, 0, 0])
        generic_pcb(wid = 34*2, len = 80);
        
        translate([0, -7, 128])
        rotate([90, 0, 0])
        //samsung_mobilephone();
        google_pxl7a_mobilephone();
        translate([+24, 9, 50])
        lipo500mAh();
    }
}

elements();

translate([+30, 0, 0])
rotate([90, 0, 90])
leg01a(0);
translate([-30, 0, 0])
rotate([90, 0, -90])
leg01a(0);

color("LightGreen")
translate([+33, 0, 41])
rotate([90, 0, 90])
sidepanel_left();