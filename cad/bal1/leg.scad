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
                    hull() {
                        translate([+0, +0, -10])
                        cylinder(r = 25, h = 10, $fn = loc_res);
                        *translate([+0, +0, -16])
                        cylinder(r = 22, h = 16, $fn = loc_res);
                    }
                    // legs
                    hull() {
                        translate([+8, 0, -2])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                        translate([+8, 0, -8])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                        translate([+12, 0, -2])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                        translate([+12, 0, -8])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                    }
                    hull() {
                        translate([-8, 0, -2])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                        translate([-8, 0, -8])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                        translate([-12, 0, -2])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                        translate([-12, 0, -8])
                        rotate([-90, 0, 0])
                        cylinder(d = 4, h = 50, $fn = loc_res);
                    }
                    // mounts to upper stage
                    hull() {
                        translate([+10, 41, -10])
                        cylinder(d = 6, h = 10, $fn = loc_res);
                        translate([-10, 41, -10])
                        cylinder(d = 6, h = 10, $fn = loc_res);
                        translate([+16, 47, -10])
                        cylinder(d = 6, h = 10, $fn = loc_res);
                        translate([-16, 47, -10])
                        cylinder(d = 6, h = 10, $fn = loc_res);
                    }
                    hull() {
                        translate([+18, 47, -10])
                        cylinder(d = 2, h = 10, $fn = loc_res);
                        translate([+18, 54, -10])
                        cylinder(d = 2, h = 10, $fn = loc_res);
                    }
                    hull() {
                        translate([-18, 47, -10])
                        cylinder(d = 2, h = 10, $fn = loc_res);
                        translate([-18, 54, -10])
                        cylinder(d = 2, h = 10, $fn = loc_res);
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
            translate([+4, +20, -17])
            cylinder(d = 4, h = 20, $fn = loc_res);
            translate([-4, +20, -17])
            cylinder(d = 4, h = 20, $fn = loc_res);
            translate([+4, +30, -17])
            cylinder(d = 4, h = 20, $fn = loc_res);
            translate([-4, +30, -17])
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
        hull() {
            translate([+10, 10, -14])
            rotate([-90, 0, 0])
            cylinder(d = 4, h = 110, $fn = loc_res);
            translate([+10, 24, -4])
            rotate([-90, 0, 0])
            cylinder(d = 4, h = 110, $fn = loc_res);
        }
        hull() {
            translate([-10, 10, -14])
            rotate([-90, 0, 0])
            cylinder(d = 4, h = 110, $fn = loc_res);
            translate([-10, 24, -4])
            rotate([-90, 0, 0])
            cylinder(d = 4, h = 110, $fn = loc_res);
        }
        
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
        translate([+16, 47, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        translate([-16, 47, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        translate([0, 47, -17])
        cylinder(d = 3.2, h = 18, $fn = loc_res);
        translate([0, 41, -17])
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
*leg01a(0, loc_res = 32);

module leg01b(show_elements = 1, loc_res = 32) {
    echo("creat a cover plate for odrive mocro");
}
*leg01b();


module mount_between_legs(wid = 20, loc_res = 32) {
    mount_wid = 4;
    middle_wid = 10;
    difference() {
        union() {
            // mounts to upper stage
            translate([0, 0, 0])
            hull() {
                translate([+10, -6, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
                translate([-10, -6, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
                translate([+16, 0, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
                translate([-16, 0, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
            }
            // mounts to upper stage
            translate([0, 0, wid-mount_wid])
            hull() {
                translate([+10, -6, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
                translate([-10, -6, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
                translate([+16, 0, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
                translate([-16, 0, -0])
                cylinder(d = 6, h = mount_wid, $fn = loc_res);
            }
            // between mounts
            hull() {
                translate([+10, -6, mount_wid])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
                translate([+9.4, -0, mount_wid])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
                translate([+6.4, -6, mount_wid])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
                translate([+4, -6, (wid-middle_wid)/2+1])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
            }
            hull() {
                translate([+10, -6, wid-mount_wid-0.1])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
                translate([+9.4, -0, wid-mount_wid-0.1])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
                translate([+6.4, -6, wid-mount_wid-0.1])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
                translate([+4, -6, (wid-middle_wid)/2+middle_wid-1])
                cylinder(d = 6, h = 0.1, $fn = loc_res);
            }
            // between mounts
            hull() {
                translate([-10, -6, mount_wid])
                cylinder(d = 6, h = wid-2*mount_wid, $fn = loc_res);
                translate([-9.4, -0, mount_wid])
                cylinder(d = 6, h = wid-2*mount_wid, $fn = loc_res);
                translate([-6.4, -6, mount_wid])
                cylinder(d = 6, h = wid-2*mount_wid, $fn = loc_res);
            }
        }
        // cut leg mounting holes + space for M3 nuts
        translate([+16, 0, -1]) {
            cylinder(d = 3.2, h = wid+2, $fn = loc_res);
            translate([0, 0, mount_wid-1])
            m3_nut_cut(wid-2*mount_wid+4);
        }
        translate([-16, 0, -1]) {
            cylinder(d = 3.2, h = wid+2, $fn = loc_res);
            translate([0, 0, mount_wid-1])
            m3_nut_cut(wid-2*mount_wid+4);
        }
        translate([0, 0, -1]) {
            cylinder(d = 3.2, h = wid+2, $fn = loc_res);
            translate([0, 0, mount_wid-1])
            m3_nut_cut(wid-2*mount_wid+4);
        }
        translate([0, -6, -1]) {
            cylinder(d = 3.2, h = mount_wid+2, $fn = loc_res);
            translate([0, 0, mount_wid-1])
            m3_nut_cut(mount_wid);
        }
        translate([0, -6, wid-mount_wid-1]) {
            cylinder(d = 3.2, h = mount_wid+2, $fn = loc_res);
            translate([0, 0, -(mount_wid-2)])
            m3_nut_cut(mount_wid);
        }
        *translate([0, -6, -1]) {
            cylinder(d = 3.2, h = wid+2, $fn = loc_res);
            translate([0, 0, mount_wid-1])
            m3_nut_cut(wid-2*mount_wid+4);
        }
    }
    // put in the middle
    hull() {
        translate([+4, -6, (wid-middle_wid)/2])
        cylinder(d = 6, h = middle_wid, $fn = loc_res);
        translate([-6.4, -6, (wid-middle_wid)/2])
        cylinder(d = 6, h = middle_wid, $fn = loc_res);
    }
}
color("LightBlue")
translate([0, -47, 0])
leg01a(0, loc_res = 18);
mount_between_legs(60);

// print 
*leg01a(0, loc_res = 128); // 1 x
*magnet_holder_8mm_6x5mm_magnet(0, loc_res = 128); // 1 x
*magnet_holder_5mm_v1_0(128); // 1 x
