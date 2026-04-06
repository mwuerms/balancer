/**
 * Martin Egli
 * 2026-03-24
 * electronics
 */

module pcbAS5600_mount_holes_cut(cut_dia = 3.5, cut_len = 10, loc_res = 32) {
    a = (19.7+12.4)/2/2;
    translate([+a,+a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
    translate([+a, -a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
    translate([-a, +a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
    translate([-a, -a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
}
//pcbAS5600_mount_holes_cut();

module pcbAS5600_pcb(pcb_th = 1.5, loc_res = 32) {
    a = 23/2;
    r1 = 3;
    hull() {
        translate([+(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([+(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
    }
}
//pcbAS5600_pcb();

module pcbAS5600_pcb_cut(pcb_th = 2, loc_res = 32) {
    a = (23+1)/2;
    r1 = 3;
    hull() {
        translate([+(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([+(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
    }
}
//pcbAS5600_pcb_cut();

module pcbAS5600_pcb_cut2(pcb_th = 2, loc_res = 32) {
    a = (23+1)/2;
    r1 = 3;
    hull() {
        translate([+(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([+(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
    }
    translate([-10/2, -24/2, 0])
    cube([10, 24, 3]);
    translate([-24/2, -10/2, 0])
    cube([24, 10, 3]);
}
//pcbAS5600_pcb_cut2();



module pcbMT6701_mount_holes_cut(cut_dia = 3.1, cut_len = 10, loc_res = 32) {
    a = (17)/2;
    translate([+a,+a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
    translate([+a, -a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
    translate([-a, +a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
    translate([-a, -a, 0])
    cylinder(d = cut_dia, h = cut_len, $fn = loc_res);
}
//pcbMT6701_mount_holes_cut();

module pcbMT6701_pcb(pcb_th = 1.5, loc_res = 32) {
    a = 23/2;
    r1 = 3;
    hull() {
        translate([+(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([+(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
    }
}
//pcbMT6701_pcb();

module pcbMT6701_pcb_cut(pcb_th = 2, loc_res = 32) {
    a = (23+1)/2;
    r1 = 3;
    hull() {
        translate([+(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([+(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), +(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
        translate([-(a-r1), -(a-r1), 0])
        cylinder(r = r1, h = pcb_th, $fn = loc_res);
    }
}
//pcbMT6701_pcb_cut();

module pcbMT6701(loc_res = 32) {
    difference() {
        union() {
            color("Gray")
            translate([0, 0, 0])
            pcbAS5600_pcb();
            // SOIC-8 4*5 mm
            color("DarkGray")
            translate([-2, -2.5, 1.5])
            cube([4, 5, 1]);
        }
        translate([0, 0, -1])
        pcbMT6701_mount_holes_cut();
    }
}
//pcbMT6701();
