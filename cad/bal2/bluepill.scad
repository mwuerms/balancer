/**
 * Martin Egli
 * 2022-09-14
 * electronics
 */

module pcbBluePill(loc_res = 32) {
    *origin();
    color("Blue") {
        translate([0, 0, 0])
        cube([53, 22.5, 1.5]);
    }
    color("Black") {
        translate([21, 11, 1])
        rotate([0, 0, -45])
        cube([8, 8, 2]);
    }
    color("Silver") {
        translate([0, 7, 1.5])
        cube([6, 8, 2]);
        
        for(n = [0:1:20-1]) {
            translate([2.3+n*2.54, 3.3, -5])
            cylinder(d = 0.6, h = 8, $fn = loc_res);
            translate([2.3+n*2.54, 3.3+6*2.54, -5])
            cylinder(d = 0.6, h = 8, $fn = loc_res);
        }
    }
}

module pcbBluePill_header_holes(loc_res = 32) {
    for(n = [0:1:20-1]) {
        translate([2.3+n*2.54, 3.3, -5])
        cylinder(d = 1, h = 8, $fn = loc_res);
        translate([2.3+n*2.54, 3.3+6*2.54, -5])
        cylinder(d = 1, h = 8, $fn = loc_res);
    }
}
