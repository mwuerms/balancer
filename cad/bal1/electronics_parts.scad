/**
 * Martin Egli
 * 2022-09-14
 * electronics
 */

use <parts.scad>

module pcbMPU9250(loc_res = 32) {
    *origin();
    color("Blue") {
        difference() {
            translate([0, 0, 0])
            cube([26, 16, 1.5]);
            translate([3, 16-3, -1])
            cylinder(d = 3, h = 4.5, $fn = loc_res);
            translate([26-3, 16-3, -1])
            cylinder(d = 3, h = 4.5, $fn = loc_res);
        }
    }
    color("Black") {
        translate([26/2-2, 16/2-2, 1])
        cube([4, 4, 2]);
    }
}

module pcbMPU9250cut(loc_res = 32) {
    hull() {
        translate([3/2, 3/2, -3])
        cylinder(d = 3, h = 5, $fn = loc_res);
        translate([26-3/2, 3/2, -3])
        cylinder(d = 3, h = 5, $fn = loc_res);
    }
}

module pcbMPU9250mounts(loc_res = 32) {
    translate([3, 16-3, 0])
    cylinder(d = 2, h = 2, $fn = loc_res);
    translate([26-3, 16-3, 0])
    cylinder(d = 2, h = 2, $fn = loc_res);
}

module pcbMPU6500(loc_res = 32) {
    origin();
    color("Blue")
    translate([0, 0, 0])
    cube([16, 14, 1]);
    color("White")
    translate([3.5, -2, 1])
    cube([10, 5.5, 4]);
    color("Gray")
    translate([7, 7, 1])
    cube([4, 4, 1]);
}

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

module pcbAS5600(loc_res = 32) {
    difference() {
        union() {
            color("White")
            translate([0, 0, 0])
            pcbAS5600_pcb();
            // SOIC-8 4*5 mm
            color("Gray")
            translate([-2, -2.5, 1.5])
            cube([4, 5, 1]);
        }
        translate([0, 0, -1])
        pcbAS5600_mount_holes_cut();
    }
}

//pcbMPU9250();
//pcbBluePill();
//pcbAS5600();

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

module lipo500mAh(col = "LightGray", loc_res = 32) {
    color(col) {
        hull() {
            translate([0, 0, 0])
            cylinder(d = 16, h = 32, $fn = loc_res);
            translate([-(62-16/2), +7, 0])
            cylinder(d = 2, h = 32, $fn = loc_res);
            translate([-(62-16/2), -7, 0])
            cylinder(d = 2, h = 32, $fn = loc_res);
        }
    }
    color("Red") {
        translate([0, -6, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, -4, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
    }
    color("Black") {
        translate([0, -2, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, -0, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, +2, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, +4, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
        translate([0, +6, 33])
        rotate([0, 90, 0])
        cylinder(d = 2, h = 12, $fn = loc_res);
    }
}
//lipo500mAh();
module lipo500mAh_cut(loc_res = 32) {
    translate([-(64-16/2), -17/2, 0])
    cube([72, 17, 34]);
}
//lipo500mAh_cut();

module rockswitch(col = "DarkGray", loc_res = 32) {
    color(col) {
        translate([2, -11/2, 4])
        rotate([0, -30, 0])
        cube([4, 11, 8]);
        translate([0, -15/2, 0])
        cube([2, 15, 21]);
        translate([-12, -13/2, 0])
        cube([12, 13, 21]);
        translate([-20, -1/2, (21-16)/2])
        cube([12, 1, 16]);
    }
}
//rockswitch();

module rockswitch_cut(loc_res = 32) {
    translate([-20, -13.4/2, -0.2])
    cube([25, 13.4, 21.4]);
}
//rockswitch_cut();

module google_pxl7a_mobilephone(col = "LightBlue", loc_res = 32) {
    color(col)
    hull() {
        translate([+70/2, +140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
        translate([+70/2, -140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
        translate([-70/2, +140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
        translate([-70/2, -140/2, -2]) {
            cylinder(r = 6, h = 12, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 8, h = 8, $fn = loc_res);
        }
    }
    // USBC
    hull() {
        translate([+5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
        translate([-5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
    }
}
//google_pxl7a_mobilephone();

module samsung_mobilephone(col = "DarkGray", loc_res = 32) {
    color(col)
    hull() {
        translate([+((74-10)/2), +((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
        translate([+((74-10)/2), -((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
        translate([-((74-10)/2), +((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
        translate([-((74-10)/2), -((146-10)/2), 0]) {
            cylinder(r = 8, h = 10, $fn = loc_res);
            translate([0, 0, 2])
            cylinder(r = 10, h = 8, $fn = loc_res);
        }
    }
    // USBC
    hull() {
        translate([+5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
        translate([-5, -140/2, 4])
        rotate([90, 0, 0])
        cylinder(d = 4, h = 10, $fn = loc_res);
    }
}
//samsung_mobilephone();

module generic_pcb(col = "LightGreen", wid = 100, len = 160, th = 1.5) {
    color(col)
    translate([0, 0, 0])
    cube([wid, len, th]);
}

//PTV09A-4030F
module poti_PTV09A_4(loc_res = 32) {
    // knob
    color("DarkGray") {
        cylinder(d = 6.8, h = 15, $fn = loc_res);
    }
    // ody
    color("LightBlue") {
        translate([-10/2, -10/2, 0])
        cube([10, 12, 6.8]);
    }
    // contacts
    color("Silver") {
        translate([-(10.6-2)/2, 0, -3.8])
        cylinder(d = 2, h = 6.8, $fn = loc_res);
        translate([+(10.6-2)/2, 0, -3.8])
        cylinder(d = 2, h = 6.8, $fn = loc_res);
        
        translate([+(5)/2, 7, -3.8])
        cylinder(d = 1, h = 6.8, $fn = loc_res);
        translate([0, 7, -3.8])
        cylinder(d = 1, h = 6.8, $fn = loc_res);
        translate([-(5)/2, 7, -3.8])
        cylinder(d = 1, h = 6.8, $fn = loc_res);
    }
}
poti_PTV09A_4();

module poti_PTV09A_4_cut(loc_res = 32) {
    translate([-(10.6-2)/2, 0, -5])
    cylinder(d = 2.5, h = 10, $fn = loc_res);
    translate([+(10.6-2)/2, 0, -5])
    cylinder(d = 2.5, h = 10, $fn = loc_res);
    
    translate([+(5)/2, 7, -5])
    cylinder(d = 1.1, h = 10, $fn = loc_res);
    translate([0, 7, -5])
    cylinder(d = 1.1, h = 10, $fn = loc_res);
    translate([-(5)/2, 7, -5])
    cylinder(d = 1.1, h = 10, $fn = loc_res);
}
//poti_PTV09A_4_cut();
