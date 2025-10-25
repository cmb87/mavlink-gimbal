include <Getriebe.scad>


///////////////////////////
//Bearing Size Parameters//
///////////////////////////
ball_diameter = 5.5; //Diameter of the balls to be used.
inner_diameter = 35; //Inside opening Diameter.
outer_diameter = 49; //The outer diameter will be the larger of this or a calculated minimum
min_wall = 1; // Minimum outside/inside wall thickness
min_ceiling = 1; //Minimum thickness of the top and bottom
ball_spacing_ratio = 1.16; // Give the balls some space.


////////////////////////////
//Bearing Shape Parameters//
////////////////////////////
Square = 0; //Set to 1 to have a square outside profile.
Square_Screw_Holes = 0; // If square=1, this can add holes for mounting.  Set it to hole diameter.
Inside_Fill_Hole = 0; // Adds a fill-hole to put the balls in to the inside part
Outside_Fill_Hole = 1; // Adds a fill-hole to put the balls in to the outside part

Flange = 1; //Set to one to add a flange.  Only applies to round bearings.
Flange_Height = 1;
Flange_Length = 5;


/////////////////////
//Output Parameters//
/////////////////////
Show_Profile = 0; // Shows a cutaway of the bearing.
Show_Inside = 1;
Show_Outside = 1;


//////////////////
//Math And Stuff//
//////////////////
ball_covered_fraction = 1/3;
ball_size = ball_diameter * ball_spacing_ratio;
ball_radius = ball_size/2;
inner_radius = inner_diameter/2;
outer_radius = outer_diameter/2;

race_radius = max((inner_radius + ball_size/2 + min_wall), inner_radius + (outer_radius-inner_radius)/2);

height = ball_diameter + min_ceiling*2;

module raceway() {
my_taurus_center = race_radius;//inner_radius + ball_size/2 + min_wall;
	rotate_extrude()
	translate([my_taurus_center,min_ceiling+ball_radius,0])
	circle(r=ball_radius);
}

module ball_fill_tube(){

	or_option = inner_radius + ball_size + min_wall*2;

	my_or = max(or_option, outer_diameter) + 1;

translate([0,0,height/2])
	rotate([0,90,0])
	cylinder(r=ball_radius,h=my_or);

}

module inner_part() {
	my_ir = inner_radius;

//	my_or = my_ir + min_wall + ball_size*ball_covered_fraction;
	my_or = race_radius - ball_diameter*ball_covered_fraction;

	difference(){
	cylinder(r=my_or, h=height);
	translate([0,0,-1])
	cylinder(r=my_ir, h=height+2);
	raceway();
	if (Inside_Fill_Hole == 1){
	ball_fill_tube();
	}}


}

module screw_holes(size) {
	sp = size*3/8;
	sm = -1*sp;
	for(i=[	[sp,sp,-1],
				[sp,sm,-1],
				[sm,sp,-1],
				[sm,sm,-1] ])
	{
		translate(i)
		cylinder(r=Square_Screw_Holes/2,h=height+2);
	}
}




module outer_part() {
	//my_ir = inner_radius + ball_size*(1-ball_covered_fraction) + min_wall;
	my_ir = race_radius + ball_size*ball_covered_fraction;

	or_option = inner_radius + ball_size + min_wall*2;

	my_or = max(or_option, outer_radius);

	if(Square==1){
		difference(){
			translate([0,0,height/2])
			cube([2*my_or,2*my_or,height],center=true);
			translate([0,0,-1])
			cylinder(r=my_ir, h=height+2);
			raceway();
			screw_holes(2*my_or);
		if (Outside_Fill_Hole == 1){
			ball_fill_tube();
			}
		}
	} else {
		difference(){
			union(){
				difference(){
					cylinder(r=my_or, h=height);
					translate([0,0,-1])
					cylinder(r=my_ir, h=height+2);
					raceway();

				}
				difference(){
					cylinder(r=my_or+Flange_Length, h=Flange_Height);
					translate([0,0,-1])
					cylinder(r=my_ir,h=Flange_Length+2);
				}
			}
		if (Outside_Fill_Hole == 1){
			ball_fill_tube();
			}
		}
}

}

module cuber(){
translate([-100,0,-20])
cube([200,50,40]);
}





//screw_holes(20);
//showit();

