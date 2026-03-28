/**
 * Maritn Egli
 *2028-03-28
 */

module g431b_esc1_pcb_model() {
    // attention this is mirrored, x pos was to the right, y pos to the top
    th = 1.6;
    mirror([1, 0, 0])
    difference() {
        union() {
            // PCB
            color("DarkBlue")
            translate([0, 0, 0])
            cube([17.75, 41, th]);
            
            // top side GTO
            // gold motor contacts
            color("Gold")
            translate([0.55, 0.1, -0.01])
            cube([4.5, 2.5, th+0.02]);
            color("Gold")
            translate([6.75, 0.1, -0.01])
            cube([4.5, 2.5, th+0.02]);
            color("Gold")
            translate([13.0, 0.1, -0.01])
            cube([4.5, 2.5, th+0.02]);
            // mosfets
            color("DarkGray")
            translate([0.7, 2.9, th])
            cube([5.2, 7.2, 1]);
            color("DarkGray")
            translate([6.35, 2.9, th])
            cube([5.2, 7.2, 1]);
            color("DarkGray")
            translate([12, 2.9, th])
            cube([5.2, 7.2, 1]);
            color("DarkGray")
            translate([0.7, 11.3, th])
            cube([5.2, 7.2, 1]);
            color("DarkGray")
            translate([6.35, 11.3, th])
            cube([5.2, 7.2, 1]);
            color("DarkGray")
            translate([12, 11.3, th])
            cube([5.2, 7.2, 1]);
            // mosfet drivers
            color("DarkGray")
            translate([0.7, 22.3, th])
            cube([5.0, 4.8, 1]);
            color("DarkGray")
            translate([6.35, 22.3, th])
            cube([5.0, 4.8, 1]);
            color("DarkGray")
            translate([12, 22.3, th])
            cube([5.0, 4.8, 1]);
            
            // gold contacts battery
            color("Gold")
            translate([0.15, 36.9, -0.01])
            cube([3.1, 4.0, th+0.02]);
            color("Gold")
            translate([14.6, 36.9, th])
            cube([3.1, 4.1, 0.01]);
            // SWD J4
            color("White")
            translate([0.2, 31.25, th])
            cube([2.3, 5.4, 1]);
            // UART J?
            color("White")
            translate([3.55, 39.25, th])
            cube([9.7, 1.6, 1]);
            
            // bottom side GBO
            // CAN J1
            color("White")
            translate([15.75, 25.6, -1])
            cube([1.85, 7.7, 1]);
            
            // HAL J8
            color("White")
            translate([3.4, 32.3, -1])
            cube([9.5, 1.85, 1]);
            
            // MCU
            color("DarkGray")
            translate([5.25, 18.95, -1])
            cube([7.1, 7.1, 1]);
            
            // current shunts
            color("LightGray")
            translate([1.65, 8.2, -0.5])
            cube([3.3, 5.0, 0.5]);
            color("LightGray")
            translate([7.3, 8.2, -0.5])
            cube([3.3, 5.0, 0.5]);
            color("LightGray")
            translate([12.95, 8.2, -0.5])
            cube([3.3, 5.0, 0.5]);
            
            // input cap bank
            color("Orange")
            for(n = [0:1:6]) {
                translate([3.3+n*1.65, 35.7, -0.8])
                cube([1.5, 1.9, 0.8]);
                translate([3.3+n*1.65, 38.6, -0.8])
                cube([1.5, 1.9, 0.8]);
            }
        }
        // cut near motor contacts
        translate([5.2, -0.2, -1])
        cube([1.5, 2.7, th+2]);
        translate([11.4, -0.2, -1])
        cube([1.5, 2.7, th+2]);
    }
}
g431b_esc1_pcb_model();

module g431b_esc1_simple_model() {
    th = 1.6;
    color("LightBlue")
    translate([-17.8, 0, -1])
    cube([17.8, 41, th+2]);
}
g431b_esc1_simple_model();

















