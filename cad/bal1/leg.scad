/**
 * Martin Egli
 * 2026-02-22
 */

use <electronics_parts.scad>
use <mechanics_parts.scad>
use <parts.scad>
use <printparts.scad>
use <screws.scad>

module elements(show_elements = 1, loc_res = 32) {
    if(show_elements) {
        translate([0, 0, 0])
        rotate([0, 0, 45])
        bldc5010_motor(0, loc_res = loc_res);
        translate([0, 0, -5.5])
        magnet_holder_8mm_6x5mm_magnet(1, loc_res = loc_res);

        translate([0, 0, -10])
        rotate([180, 0, 0])
        odrive_micro(loc_res = loc_res);
        
        translate([+13, +13, -2])
        m3_nut();
        translate([+13, -13, -2])
        m3_nut();
        translate([-13, +13, -2])
        m3_nut();
        translate([-13, -13, -2])
        m3_nut();
    }
}

module leg01a(show_elements = 1, loc_res = 32) {
    elements(show_elements);
    
    difference() {
        union() {
            difference() {
                union() {
                    translate([+0, +0, -16])
                    cylinder(r = 24, h = 16, $fn = loc_res);
                    // legs
                    hull() {
                        translate([+10, 0, -4])
                        rotate([-90, 0, 0])
                        cylinder(d = 8, h = 60, $fn = loc_res);
                        translate([+10, 0, -12])
                        rotate([-90, 0, 0])
                        cylinder(d = 8, h = 60, $fn = loc_res);
                    }
                    hull() {
                        translate([-10, 0, -4])
                        rotate([-90, 0, 0])
                        cylinder(d = 8, h = 60, $fn = loc_res);
                        translate([-10, 0, -12])
                        rotate([-90, 0, 0])
                        cylinder(d = 8, h = 60, $fn = loc_res);
                    }
                    // mounts to upper stage
                    hull() {
                        translate([+10, 50, -16])
                        cylinder(d = 8, h = 16, $fn = loc_res);
                        translate([-10, 50, -16])
                        cylinder(d = 8, h = 16, $fn = loc_res);
                        translate([+16, 56, -16])
                        cylinder(d = 8, h = 16, $fn = loc_res);
                        translate([-16, 56, -16])
                        cylinder(d = 8, h = 16, $fn = loc_res);
                    }
                    
                }
                translate([0, 0, -17])
                hull() {
                    translate([+13, +13, -0])
                    cylinder(d = 7, h = 7, $fn = loc_res);
                    translate([+13, -13, -0])
                    cylinder(d = 7, h = 7, $fn = loc_res);
                    translate([-13, +13, -0])
                    cylinder(d = 7, h = 7, $fn = loc_res);
                    translate([-13, -13, -0])
                    cylinder(d = 7, h = 7, $fn = loc_res);
                }
                translate([-26/2, -26/2, -11])
                cube([26, 26, 6]);
            }
            translate([+13, +13, -10])
            cylinder(d = 6, h = 5, $fn = loc_res);
            translate([+13, -13, -10])
            cylinder(d = 6, h = 5, $fn = loc_res);
            translate([-13, +13, -10])
            cylinder(d = 6, h = 5, $fn = loc_res);
            translate([-13, -13, -10])
            cylinder(d = 6, h = 5, $fn = loc_res);
        }
        // cut motor mount screws
        translate([0, 0, -3])
        rotate([0, 0, 45])
        bldc5010_m3cut_stator();
        // cut middle hole
        translate([0, 0, -6])
        cylinder(d = 9.5, h = 10, $fn = loc_res);
        // M3 cuts
        translate([+13, +13, -2])
        m3_nut_bolt_cut(4, 20, m3_nut_dia = 7);
        translate([+13, -13, -2])
        m3_nut_bolt_cut(4, 20, m3_nut_dia = 7);
        translate([-13, +13, -2])
        m3_nut_bolt_cut(4, 20, m3_nut_dia = 7);
        translate([-13, -13, -2])
        m3_nut_bolt_cut(4, 20, m3_nut_dia = 7);
        // cutout for motor cables
        hull() {
            translate([+4, +18, -17])
            cylinder(d = 4, h = 20, $fn = loc_res);
            translate([-4, +18, -17])
            cylinder(d = 4, h = 20, $fn = loc_res);
        }
        hull() {
            translate([+5, +17, -17])
            cylinder(d = 8, h = 7, $fn = loc_res);
            translate([-5, +17, -17])
            cylinder(d = 8, h = 7, $fn = loc_res);
            translate([+13, +13, -17])
            cylinder(d = 7, h = 7, $fn = loc_res);
            translate([-13, +13, -17])
            cylinder(d = 7, h = 7, $fn = loc_res);
        }
        // cutput cables in legs
        *translate([+10, 0, -12])
        rotate([-90, 0, 0])
        cylinder(d = 4, h = 110, $fn = loc_res);
        *translate([-10, 0, -12])
        rotate([-90, 0, 0])
        cylinder(d = 4, h = 110, $fn = loc_res);
        
        hull() {
            translate([+10, 0, -14])
            rotate([-90, 0, 0])
            cylinder(d = 3, h = 110, $fn = loc_res);
            translate([+10, 0, -11.5])
            rotate([-90, 0, 0])
            cylinder(d = 3, h = 110, $fn = loc_res);
        }
        hull() {
            translate([-10, 0, -14])
            rotate([-90, 0, 0])
            cylinder(d = 3, h = 110, $fn = loc_res);
            translate([-10, 0, -11.5])
            rotate([-90, 0, 0])
            cylinder(d = 3, h = 110, $fn = loc_res);
        }
        // cut leg mounting holes
        translate([+16, 56, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        translate([-16, 56, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        translate([0, 56, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        translate([0, 50, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        
        // cut out usbc
        hull() {
            translate([-25, -9, -13.5])
            rotate([0, 90, 0])
            cylinder(d = 7, h = 12, $fn = loc_res);
            translate([-25, -9, -18])
            rotate([0, 90, 0])
            cylinder(d = 7, h = 12, $fn = loc_res);
            translate([-25, -3, -18])
            rotate([0, 90, 0])
            cylinder(d = 7, h = 12, $fn = loc_res);
            translate([-25, -3, -13.5])
            rotate([0, 90, 0])
            cylinder(d = 7, h = 12, $fn = loc_res);
        }
        // can connector
        translate([11, -11.5, -17])
        cube([10, 22, 5.5]);
        
        // see inside
        *translate([0, 0, -50])
        cube(100);
    }
    
}
*leg01a(0, loc_res = 64);

// print 
leg01a(0, loc_res = 128); // 1 x
*magnet_holder_8mm_6x5mm_magnet(0, loc_res = 128); // 1 x
*magnet_holder_5mm_v1_0(128); // 1 x
