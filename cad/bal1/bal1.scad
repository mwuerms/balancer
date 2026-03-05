/**
 * Martin Egli
 * 2026-02-22
 */
 use <leg.scad>
 use <electronics_parts.scad>
 
 module elements(show_elements = 1, loc_res = 32) {
    if(show_elements) {
        translate([-34, +2, 87])
        rotate([0, 90, 0])
        pcbMPU9250();
        
        translate([-34, 0, 60])
        rotate([90, 0, 0])
        generic_pcb(wid = 34*2, len = 80);
        
        translate([0, -10, 138])
        rotate([90, 0, 0])
        samsung_mobilephone();
        translate([+24, 10, 61])
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
 
 