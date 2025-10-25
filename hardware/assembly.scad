include <Getriebe.scad>
include <kugellager.scad>
include <airframe/gimbalBay.scad>
include <airframe/innerShell.scad>
include <airframe/outerShell.scad>
include <airframe/bulkheads.scad>
include <airframe/zholes.scad>




//======================================
module shell(){
  difference(){
      outerShell();
      innerShell();
  }    
    
}

//======================================
if (false)
intersection(){
gimbalBay();
shell();
}

//bulkhead_5();
zholes_5(ay=1.1); 

//bulkhead_2();
zholes_2(ay=1.1); 


l = 82;

// ------------------------
color("red")
difference(){
intersection(){
translate([0,-25.5,l/2])
cube([110,3,l],center=true);
outerShell();  
}
translate([0,-20,47])
rotate([90,0,0])
cylinder(r=52.1/2,h=10,$fn=50);

}


// ------------------------
translate([0,-23,47])
rotate([90,0,0])

union(){
    hohlrad(modul=1, zahnzahl=31, breite=5, randbreite=1, eingriffswinkel = 20, schraegungswinkel = 0);


    if (Show_Profile ==1){
        if (Show_Inside ==1){
        difference(){
        inner_part();
        cuber();
        }
    }

        if (Show_Outside == 1){
        difference(){
        outer_part();
        cuber();
        }
    }
    } else {
        if (Show_Inside ==1){
        inner_part();
    }
        if (Show_Outside == 1){
        outer_part();
    }
    }

}