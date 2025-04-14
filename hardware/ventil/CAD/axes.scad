$fa = 6;
$fs = 0.2;
$fn = 100;

d_centre=1;
d_axe=5;
d_tete=8;
h_axe=7.8;
h_tete=0;

difference() {
    union() {
	cylinder(d = d_tete, h=h_tete);
        cylinder(d = d_axe, h=h_tete+h_axe);
    };
    translate([0,0,-1]) cylinder(d = d_centre, h=h_tete+h_axe+2);
};
