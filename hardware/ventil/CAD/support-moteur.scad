$fa = 6;
$fs = 0.2;
$fn = 100;

offset_axe=8;
support_xp=17;
support_xm=8;
support_y=44;
support_h=10;
ep_plexi=6;
dia_inserts=5;
entraxe_inserts=35;
dia_centrage_axe=9.2;

module support_base() {
    difference() {
	translate([-support_xm-offset_axe, -support_y/2, 0]) 
	    cube([support_xp+support_xm, support_y, support_h]);
	translate([0, -entraxe_inserts/2, -1]) cylinder(d=dia_inserts, h=support_h+2);
	translate([0, entraxe_inserts/2, -1]) cylinder(d=dia_inserts, h=support_h+2);
	translate([-offset_axe, 0, -1]) cylinder(d=dia_centrage_axe, h=support_h+2);
    };
};

module support() {
    difference() {
        translate([offset_axe, 0, 0]) support_base();
        linear_extrude(height=ep_plexi) import(file="passage_bielles.dxf");
    };
};

support();
// projection(cut = true) translate([0,0,-1]) support();
