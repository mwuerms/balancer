/**
 * Martin Egli
 * 2026-03-24
 * electronics
 */

module odrive_micro_mount_cuts(th = 1.6, dia1 = 3.4, loc_res = 32) {
    translate([+13, +13, -1])
    cylinder(d = dia1, h = th + 2, $fn = loc_res);
    translate([+13, -13, -1])
    cylinder(d = dia1, h = th + 2, $fn = loc_res);
    translate([-13, +13, -1])
    cylinder(d = dia1, h = th + 2, $fn = loc_res);
    translate([-13, -13, -1])
    cylinder(d = dia1, h = th + 2, $fn = loc_res);
}

module odrive_micro(loc_res = 32) {
    // PCB
    th = 1.6;
    r1 = 3;
    difference() {
        union() {
            color("Green")
            hull() {
                translate([+13, +13, 0])
                cylinder(r = r1, h = th, $fn = loc_res);
                translate([+13, -13, 0])
                cylinder(r = r1, h = th, $fn = loc_res);
                translate([-13, +13, 0])
                cylinder(r = r1, h = th, $fn = loc_res);
                translate([-13, -13, 0])
                cylinder(r = r1, h = th, $fn = loc_res);
            }
            // connectors
            color("Blue")
            translate([-16, -9.35, th])
            cube([4, 9.5, 4.35]);
            color("Blue")
            translate([12, 0.875, th])
            cube([4, 9.5, 4.35]);
            color("Blue")
            translate([12, -9.35, th])
            cube([4, 9.5, 4.35]);
            color("Blue")
            translate([-9.125, 16-4, th])
            cube([18.25, 4, 4.35]);
            // USB C
            color("LightGray")
            hull() {
                translate([-16, 3, th+3.5/2])
                rotate([0, 90, 0])
                cylinder(d = 3.5, h = 8, $fn = loc_res);
                translate([-16, 8, th+3.5/2])
                rotate([0, 90, 0])
                cylinder(d = 3.5, h = 8, $fn = loc_res);
            }
            // angle sensor
            color("DarkGray")
            translate([-1.6, -1.6, -1])
            cube([3.2, 3.2, 2]);
            
        }
        // cuts
        odrive_micro_mount_cuts();
    }
}
//odrive_micro();