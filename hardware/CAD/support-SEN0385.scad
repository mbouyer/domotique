 $fa = 6;
 $fs = 0.2;
 $fn = 100;

module support_SEN0385()
{
    d_ext=20;
    d_hexa=17.32;
    h_hexa=20;
    ep_colerette=2;
    l_hexa=(d_hexa/2) * cos(30);

    translate([d_hexa/2+1, 0, 0]) {
	difference() {
	  union() {
	      cylinder(h=h_hexa-ep_colerette*2, d=d_ext, center=true);
	      translate([0,0,-h_hexa/2+ep_colerette/2])
	          cylinder(h=ep_colerette, d=d_ext+ep_colerette*2, center=true);
	      translate([0,0,+h_hexa/2-ep_colerette/2])
	          cylinder(h=ep_colerette, d=d_ext+ep_colerette*2, center=true);
	  };
	  cylinder(h=h_hexa+1, d=d_hexa, center=true, $fn=6);
	  translate([l_hexa/2, -(d_ext+ep_colerette*2)/2, -h_hexa/2-1])
	      cube([d_ext, d_ext+ep_colerette*2, h_hexa+2]);
	};
    };
};

minko=2;
h1_bras=30-minko;
h2_bras=10-minko;
l_bras=40-minko;
e_bras=3-minko;
h_plaque=70-minko;
l_plaque=50-minko;
e_plaque=e_bras;

module passage_collier(l=8,h=3) {
     rotate([90,0,0]) {
	hull() {
	    translate([-l/2-h/2, 0, 0]) cylinder(h=e_bras+minko+2, d=h, center=true);
	    translate([l/2+h/2, 0, 0]) cylinder(h=e_bras+minko+2, d=h, center=true);
	};
    };
};

d_chanfrin=8-minko/2; // taille_ecrous/cos(30) = 15/2/cos(30) - minko/2

module _bras1() {
	hull() {
	    translate([d_chanfrin/2, 0, 0]) cylinder(d=d_chanfrin, h=h1_bras+2, center=true);
	    translate([l_bras-d_chanfrin/2, 0, 0]) cylinder(d=d_chanfrin, h=h1_bras+2, center=true);
	}
};
	

module bras() {
    difference() {
        rotate([90, 0, 0]) linear_extrude(height=e_bras+d_chanfrin, center=true)
	    polygon(points=[
		[0, h2_bras/2],
		[l_bras, h2_bras/2],
		[l_bras, -h2_bras/2],
		[l_bras-3, -h2_bras/2],
		[0, h2_bras/2 - h1_bras]
	    ]);
	translate([0, d_chanfrin/2+e_bras/2, h2_bras/2-h1_bras/2]) _bras1();
	translate([0, -(d_chanfrin/2+e_bras/2), h2_bras/2-h1_bras/2]) _bras1();
    };
};

module base() {
    minkowski() {
	union() {
            translate([-e_plaque/2, 0, -10])
	        cube([e_plaque, l_plaque, h_plaque], center=true);
            bras();
	};
	sphere(d=minko);
    };
};

module support() {
    difference() {
        base();
	translate([l_bras/2, 0, 0]) passage_collier();
    }
    translate([l_bras-1.5,0,0]) support_SEN0385();
};

support();
// support_SEN0385();
//projection(cut=true) support();
