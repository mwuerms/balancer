/**
 * Martin Egli
 * 2026-02-22
 */

use <electronics_parts.scad>
use <mechanics_parts.scad>
use <parts.scad>
use <printparts.scad>
use <screws.scad>
use <g431b_esc1.scad>
use <rotarysensors.scad>

module elements(show_elements = 1, loc_res = 32) {
    if(show_elements) {
        translate([0, 0, 0])
        rotate([0, 0, 45])
        bldc5010_motor(0, loc_res = loc_res);
        translate([0, 0, -5.5])
        magnet_holder_8mm_6x5mm_magnet(1, loc_res = loc_res);
        
        *translate([0, 0, -10])
        pcbMT6701();
        
        translate([17.8/2, 40, -3])
        g431b_esc1_pcb_model();
    }
}
elements();

module leg1(show = 1, loc_res = 32) {
    difference() {
        union() {
            hull() {
                translate([0, 0, -7])
                cylinder(d = 25, h = 7, $fn = loc_res);
                translate([+(25-5)/2, 100, -7])
                cylinder(d = 5, h = 7, $fn = loc_res);
                translate([-(25-5)/2, 100, -7])
                cylinder(d = 5, h = 7, $fn = loc_res);
            }
        }
        // cut space for B431G-ESC1
        translate([-18.4/2, 39, -4])
        cube([18.4, 43, 10]);
        // motor cables
        hull() {
            translate([-17/2, 38, -1.5])
            cube([17, 2, 6]);
            translate([-10/2, 20, 1])
            cube([10, 1, 4]);
        }
        // SWD
        translate([-(18/2-8), 70.5, -1.8])
        cube([18, 6.5, 6]);
        // CAN
        translate([-(18/2)-2, 65, -4])
        cube([3, 9, 10]);
        // power + UART connection
        translate([-18/2, 81, -1.5])
        cube([18, 19, 6]);
    }    
    elements(show_elements = show, loc_res = loc_res);
}
//leg1();

// try as5601 position sensor with i2c + quadrature output
/*rotate([0, 0, 45])
bldc5010_motor(0, loc_res = 32);
translate([0, 0, -6])
soic8();*/