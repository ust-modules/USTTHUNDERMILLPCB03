$fn=80;

pcb_diameter = 80+0.2;
center_hole = 7.9*2;

rantl = 2;
thickness = 15;
pin_height = 5; // v cca prostredni urovni zmacknuti


M3_screw_diameter = 3.2;
M3_nut_diameter = 6;


// Pogo pin pro programovani

module pin(){
    cube([4, 7, 15], center=true);
    translate([0, 0, 5])
        cube([5.1, 7.7, 10], center=true);
    for(y = [-2.54, 0, 2.54]) for(x=[-2.54/2, 2.54/2])
        translate([x, y, 0])
            cylinder(d=2, h=15, center=true, $fn=30);
    
    translate([-50+2, -2.5, -13])
        cube([50, 5, 10]);
    
    // naznaceni optimalni vysky 
    //%translate([0, 0, 5.5]) square(10, center=true);
}
//translate([0, 0, 30])  pin();

difference(){
    cylinder(d = pcb_diameter + 2*rantl, h=thickness);

    difference(){
        translate([0, 0, thickness-pin_height]) cylinder(d = pcb_diameter, h=10);
        
        cylinder(d = center_hole, h=30);
        
        // do montaznich otvoru
        translate([-7, 13.9, 0]) cylinder(d=2.2, h=18);
        translate([-22.6, 0, 0]) cylinder(d=2.2, h=18);
        translate([-7, -13.9, 0]) cylinder(d=2.2, h=18);
    }
    
    // zarez na pin
    translate([-10, 0, thickness-pin_height+10-1]) cube([10, 5, 20], center=true);
    
    
    // zapusteni na senzory polohy
    translate([-10.84, 18.787, thickness-pin_height-2]) cylinder(d=6, h=30);
    translate([5.719, 15.687, thickness-pin_height-2]) cylinder(d=6, h=30);
    
    translate([11.822, -13.826, thickness-pin_height - 5.5])
        rotate([0, 0, 90]) pin();
    
    // otvor pro sesroubovani
    translate([0, 0, 10+0.2]) cylinder(d = M3_screw_diameter, h=30);
    cylinder(d = M3_nut_diameter, h=10, $fn=6);
}