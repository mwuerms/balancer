/**
 * Martin Egli
 * 2026-02-22
 */
use <leg.scad>
use <electronics_parts.scad>

module side_panel(loc_res = 32) {
    hull() {
        translate([0, 10, 0])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, 0, 0])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, 0, 6])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, 16, 6])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, -0, 110])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, 16, 110])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, -0, 0])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
    }
}

module front_panel(th = 2, loc_res = 32)  {
    difference() {
        hull() {
            translate([0, -1, 14])
            rotate([90, 0, 0])
            cylinder(d = 3, h = th, $fn = loc_res);
            translate([60, -1, 14])
            rotate([90, 0, 0])
            cylinder(d = 3, h = th, $fn = loc_res);
            translate([0, -1, 110])
            rotate([90, 0, 0])
            cylinder(d = 3, h = th, $fn = loc_res);
            translate([60, -1, 110])
            rotate([90, 0, 0])
            cylinder(d = 3, h = th, $fn = loc_res);
        }
        // cut a lot of holes to save space
            translate([30, 0, 50])
            rotate([90, 0, 0])
            cylinder(d = 20, h = th+2, $fn = loc_res);
    }
}

module middle_horizontal_panels(th = 2, loc_res = 32)  {
    difference() {
        hull() {
            translate([0, 17, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([0, -1, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([60, 17, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([60, -1, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
        }
        // cut a lot of holes to save space
        translate([30, 18/2-1, -1])
        cylinder(d = 16, h = th+2, $fn = loc_res);
    }
}

module upper_horizontal_panels(th = 2, loc_res = 32)  {
    difference() {
        hull() {
            translate([0, 17, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([0, -1, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([60, 17, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([60, -1, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
        }
        // cut holes for elements
        *translate([30, 18/2-1, -1])
        cylinder(d = 16, h = th+2, $fn = loc_res);
    }
}

module upper_cage(loc_res = 32) {
    difference() {
        union() {
            // left side
            translate([30, 0, 0])
            side_panel(loc_res = loc_res);
            // right side
            translate([-33, 0, 0])
            side_panel(loc_res = loc_res);
            
            // front panel
            translate([-30, 0, 0])
            front_panel(loc_res = loc_res);
            
            // between batteries panel
            translate([-30, 0, 44])
            middle_horizontal_panels(loc_res = loc_res);
            // middle panel
            translate([-30, 0, 44+37])
            middle_horizontal_panels(loc_res = loc_res);
            
            // upper panel
            translate([-30, 0, 44+37+30])
            upper_horizontal_panels(loc_res = loc_res);
        }
        // cut m3 holes leg mount
        translate([-50, 0, 0])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
        translate([-50, 16, 6])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
        translate([-50, 0, 6])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
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
        translate([+24, 9, 87])
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
translate([0, 0, 41])
upper_cage();