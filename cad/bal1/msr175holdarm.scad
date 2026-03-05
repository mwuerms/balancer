/**
 * Martin Egli
 * 2026-02-22
 */
use <mechanics_parts.scad>

module msr175mounts(loc_res = 32) {
    // MSR175 M5
    translate([+36.5/2, -6, -5])
    cylinder(d = 5.2, h = 20, $fn = loc_res);
    translate([-36.5/2, -6, -5])
    cylinder(d = 5.2, h = 20, $fn = loc_res);
}
module hold2(th = 4, loc_res = 32) {
    rad1 = 80;
    difference() {
        union() {
            hull() {
                translate([+36.5/2, +(rad1+6), ])
                cylinder(d = 16, h = th, $fn = loc_res);
                translate([-36.5/2, +(rad1+6), 0])
                cylinder(d = 16, h = th, $fn = loc_res);
            }
            hull() {
                translate([+36.5/2, -(rad1+6), 0])
                cylinder(d = 16, h = th, $fn = loc_res);
                translate([-36.5/2, -(rad1+6), 0])
                cylinder(d = 16, h = th, $fn = loc_res);
            }
            hull() {
                translate([0, +(rad1+6), 0])
                cylinder(d = 16, h = th, $fn = loc_res);
                translate([0, -(rad1+6), 0])
                cylinder(d = 16, h = th, $fn = loc_res);
            }
            translate([0, 0, 0])
            rotate([0, 0, 30])
            cylinder(d = 22, h = th+3, $fn = loc_res);
        }
        // cuts
        #translate([0, +(rad1+0), 0])
        rotate([0, 0, 180])
        msr175mounts(loc_res);
        #translate([0, -(rad1+0), 0])
        msr175mounts(loc_res);
        
        // bldc mount
        translate([0, 0, th+1])
        rotate([0, 0, 45])
        bldc5010_m3cut_rotor(0, loc_res = loc_res);
        translate([0, 0, -1])
        cylinder(d = 4.1, h = 3.4, $fn = loc_res);
        
        // see inside
        *cube(100);
    }
    
    
}
hold2();
