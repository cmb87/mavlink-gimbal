
//======================================
module gimbalBay(){
hull(){
translate([0,-60,103])
rotate([0,90,0])
translate([0,0,-100])
cylinder(r=3,h=200,$fn=30);

translate([0,-27,83])
rotate([0,90,0])
translate([0,0,-100])
cylinder(r=3,h=200,$fn=30);

translate([0,-27,0])
rotate([0,90,0])
translate([0,0,-100])
cylinder(r=3,h=200,$fn=30);
    
translate([0,-60,0])
rotate([0,90,0])
translate([0,0,-100])
cylinder(r=3,h=200,$fn=30);
}
}