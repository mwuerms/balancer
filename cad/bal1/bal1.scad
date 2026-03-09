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
        cylinder(d = 16, h = th+2, $fn = loc_res);
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
        // cut cable shaft through all
        translate([+2, 1, -1])
        cylinder(d = 6, h = th+2, $fn = loc_res);
        translate([+58, 1, -1])
        cylinder(d = 6, h = th+2, $fn = loc_res);
        
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

module component_door(loc_res = 32) {
    hull() {
        translate([(2+0.5), 0, -2])
        rotate([90, 0, 90])
        cylinder(d = 2, h = 59, $fn = loc_res);
        translate([(2+0.5), 29, -2])
        rotate([90, 0, 90])
        cylinder(d = 2, h = 59, $fn = loc_res);
    }    
    // hinges on the left + right
    translate([(2+0.5), 0, 0])
    rotate([90, 0, 90]) {
        cylinder(d = 6, h = 2, $fn = loc_res);
        translate([0, 0, -2.5])
        cylinder(d = 3, h = 4, $fn = loc_res);
    }
    translate([60-0.5, 0, 0])
    rotate([90, 0, 90]) {
        cylinder(d = 6, h = 2, $fn = loc_res);
        translate([0, 0, 0.5])
        cylinder(d = 3, h = 4, $fn = loc_res);
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
            translate([-30, 0, 48])
            middle_horizontal_panels(loc_res = loc_res);
            // middle panel
            translate([-30, 0, 48+37])
            middle_horizontal_panels(loc_res = loc_res);
            
            // upper panel
            translate([-30, 0, 48+37+30])
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
        // cut holes to mount component door on the back
        translate([-50, 16, 17.3])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
        
        // cut out hole for batteries
        translate([+20, -1, 50])
        cube([16, 18, 35]);
        translate([+20, -1, 128-41])
        cube([16, 18, 35]);
        
        // cut out hole for rockswitch
        translate([+33, 6, 64-41])
        rockswitch_cut();
    }
    
    translate([-32, 16, 17.3])
    rotate([90, 0, 0])
    component_door(loc_res = 32);
}
 
module elements(show_elements = 1, loc_res = 32) {
    if(show_elements) {
        translate([-30, +0, 88])
        rotate([0, 90, 0])
        pcbMPU9250();
        
        *translate([-59/2, 1, 50])
        rotate([90, 0, 0])
        generic_pcb(wid = 59, len = 30, col = "Salmon");
        
        *translate([0, -7, 128])
        rotate([90, 0, 0])
        //samsung_mobilephone();
        google_pxl7a_mobilephone();
        
        translate([+25, 9, 91])
        lipo500mAh();
        translate([+25, 9, 128])
        lipo500mAh();
        translate([+27, 17, 86])
        rotate([-90, 0, 180])
        pcbBluePill(loc_res = loc_res);
        
        translate([+33, 6, 64])
        rockswitch();
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