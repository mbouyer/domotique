$fa = 6;
$fs = 0.2;
$fn = 100;
ep_plexi=6.2;
ep_support=3;
dia_axe = 4;
longueur=60;
largeur=15;
base=20;
x_axe=8.5;
dia_centrage=5;
h_centrage=3;

longueur_ht=longueur+base;
largeur_ht=largeur+base;
hauteur_ht = ep_plexi+ep_support;

module support() {
    difference() {
        translate([-x_axe, -x_axe, 0]) {
            difference() {
	        translate([-base, -base, 0]) cube([longueur_ht, largeur_ht, hauteur_ht]);
	        cube([longueur+1, largeur+1, ep_plexi]);
	        translate([-base-1, -base-1, ep_support]) cube([longueur_ht+2, base-ep_support+1, hauteur_ht]);
	        translate([-base-1, -base-1, ep_support]) cube([base-ep_support+1, largeur_ht+2, hauteur_ht]);
	        l_vis = (base+ep_support)/2;
	        translate([25, -(l_vis)/2, hauteur_ht/2]) cube([15, l_vis, hauteur_ht+2], center=true);
            };
        };
        translate([0,0,-h_centrage]) cylinder(d=dia_axe, h=hauteur_ht+h_centrage+1);
    }
    translate([45,-25,-h_centrage]) cylinder(d=dia_axe, h=h_centrage);
};

support();
//projection(cut = true) translate([0,0,-1]) support();
//projection(cut = true) translate([0,0,1]) support();

