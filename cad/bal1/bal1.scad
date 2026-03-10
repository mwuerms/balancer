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
        translate([0, -0, 125])
        rotate([90, 0, 90])
        cylinder(d = 6, h = 3, $fn = loc_res);
        translate([0, 16, 125])
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
            translate([0, -1, 125])
            rotate([90, 0, 0])
            cylinder(d = 3, h = th, $fn = loc_res);
            translate([60, -1, 125])
            rotate([90, 0, 0])
            cylinder(d = 3, h = th, $fn = loc_res);
        }
        
        // cut a lot of holes to save space
        /*for(n = [0:1:5])
        translate([n*15, 0, 18])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 23])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 28])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 33])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 38])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 43])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 48])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 53])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 58])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 63])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 68])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 73])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 78])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 83])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 88])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 93])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 98])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 103])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 108])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 113])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 0, 118])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15-7.5, 0, 123])
        rotate([90, 0, 0])
        cylinder(d = 7, h = 10, $fn = loc_res);*/
    }
}

module middle_horizontal_panels(th = 2, loc_res = 32)  {
    difference() {
        hull() {
            translate([0, 17, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([0, 1, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([60, 17, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
            translate([60, 1, 0])
            cylinder(d = 4, h = th, $fn = loc_res);
        }
        // cut cable shaft through all -> done
        translate([2.5, 2, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
        translate([60-2.5, 2, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
        // cut a lot of holes to save space
        /*for(n = [0:1:5])
        translate([n*15, 0, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [1:1:4])
        translate([n*15-7.5, 5, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [0:1:5])
        translate([n*15, 10, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
        for(n = [1:1:4])
        translate([n*15-7.5, 14, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);*/
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
        translate([+30+20, 9, 0])
        rotate([0, 0, 180])
        poti_PTV09A_4_cut();
        translate([+30+0, 9, 0])
        rotate([0, 0, 180])
        poti_PTV09A_4_cut();
        translate([+30-20, 9, 0])
        rotate([0, 0, 180])
        poti_PTV09A_4_cut();
        
        // cut cable shaft through all -> done
        translate([2.5, 2, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
        translate([60-2.5, 2, -4])
        cylinder(d = 7, h = 10, $fn = loc_res);
    }
}

module component_door(loc_res = 32) {
    difference() {
        union() {
            hull() {
                translate([(2+0.5), 0, -2])
                rotate([90, 0, 90])
                cylinder(d = 2, h = 59, $fn = loc_res);
                translate([(2+0.5), 29, -2])
                rotate([90, 0, 90])
                cylinder(d = 2, h = 59, $fn = loc_res);
            }    
            translate([(2), 29, -2])
            rotate([90, 0, 90])
            cylinder(d = 2, h = 60, $fn = loc_res);
            
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
        // cut holes to reduce material
        /*for(n = [1:1:4])
        translate([n*15-5, 7, -4])
        cylinder(d = 8, h = 10, $fn = loc_res);
        for(n = [1:1:3])
        translate([n*15-5+15/2, (23-7)/2+7, -4])
        cylinder(d = 8, h = 10, $fn = loc_res);
        for(n = [1:1:4])
        translate([n*15-5, 23, -4])
        cylinder(d = 8, h = 10, $fn = loc_res);*/
    }
}

module upper_cage(show_door = 1, loc_res = 32) {
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
            translate([-30, 0, 126])
            upper_horizontal_panels(loc_res = loc_res);
        }
        // cut m3 holes leg mount
        translate([-50, 0, 0])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
        translate([31.2, 0, 0])
        rotate([90, 0, 90])
        cylinder(d2 = 6.2, d1 = 3.1, h = 2, $fn = loc_res);
        translate([-31.2-2, 0, 0])
        rotate([90, 0, 90])
        cylinder(d1 = 6.2, d2 = 3.1, h = 2, $fn = loc_res);
       
        translate([-50, 16, 6])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
        translate([31.2, 16, 6])
        rotate([90, 0, 90])
        cylinder(d2 = 6.2, d1 = 3.1, h = 2, $fn = loc_res);
        translate([-31.2-2, 16, 6])
        rotate([90, 0, 90])
        cylinder(d1 = 6.2, d2 = 3.1, h = 2, $fn = loc_res);
        
        translate([-50, 0, 6])
        rotate([90, 0, 90])
        cylinder(d = 3.2, h = 100, $fn = loc_res);
        translate([31.2, 0, 6])
        rotate([90, 0, 90])
        cylinder(d2 = 6.2, d1 = 3.1, h = 2, $fn = loc_res);
        translate([-31.2-2, 0, 6])
        rotate([90, 0, 90])
        cylinder(d1 = 6.2, d2 = 3.1, h = 2, $fn = loc_res);
        
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
    
    if(show_door) {
        translate([-32, 16, 17.3])
        rotate([90, 0, 0])
        component_door(loc_res = 32);
    }
}
 
module elements(show_elements = 1, loc_res = 32) {
    if(show_elements) {
        translate([-30, +0, 88])
        rotate([0, 90, 0])
        pcbMPU9250();
        
        *translate([-59/2, 1, 50])
        rotate([90, 0, 0])
        generic_pcb(wid = 59, len = 30, col = "Salmon");
        
        translate([0, -7, 128])
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
        
        // PID
        translate([+20, 9, 170])
        rotate([0, 0, 180])
        poti_PTV09A_4();
        translate([0, 9, 170])
        rotate([0, 0, 180])
        poti_PTV09A_4();
        translate([-20, 9, 170])
        rotate([0, 0, 180])
        poti_PTV09A_4();
    }
}

module puttogether(loc_res = 32) {
    elements();

    translate([+30, 0, 0])
    rotate([90, 0, 90])
    leg01a(0, loc_res = loc_res);

    color("Salmon")
    translate([20, 0, 47])
    rotate([90, 0, -90])
    mount_between_legs(40, loc_res = loc_res);

    translate([-30, 0, 0])
    rotate([90, 0, -90])
    leg01a(0, loc_res = loc_res);

    color("LightGreen")
    translate([0, 0, 41])
    upper_cage(loc_res = loc_res);
}
puttogether(loc_res = 32);

// print
*upper_cage(0, loc_res = 128); // 1 x
*component_door(loc_res = 128); // 1 x
*leg01a(0, loc_res = 128); // 2 x
*mount_between_legs(40, loc_res = 128); // 1 x
